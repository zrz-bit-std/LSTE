"""One focused responsibility extracted from the legacy GP frontier node."""

from time import time

import cv2
import numpy as np
import ros_numpy
from scipy import stats

from gp_frontier_sparse_gp import SGP2D

class GpFrontierModelMixin:
    def gp_nav_fit(self, ls1, ls2, var, alpha, noise, noise_var):
        """用当前球面观测创建 Sparse GP 模型。

        输入 ``gp_nav_din`` 的每行是 ``[theta, alpha]``，``gp_nav_dout`` 是
        该方向的占据值。随后 step() 会用此模型预测 ``gp_grd`` 全网格，从
        方差中找尚未观测充分的 frontier。
        """
        self.gp_nav = SGP2D()
        self.gp_nav.set_kernel_param(ls1, ls2, var, alpha, noise, noise_var)
        self.gp_nav.set_training_data(self.gp_nav_din, self.gp_nav_dout)
        self.gp_nav.set_indpts_from_training_data(
            self.gp_nav_indpts_sz, self.gp_nav_din)  #(indpts_size, data_size)
        self.gp_nav.set_sgp_model()
        self.gp_nav.select_trainable_param()
        self.gp_nav.minimize_loss()
        # self.gp_nav.adam_optimize_param()

    """ @brief: callback funtion to process the occupancy surface"""

    def pose_pcl_cb(self, pose_pcl_msg):
        #################### GP Nav ####################
        print("\n\n\n##### OBSV: seq=", pose_pcl_msg.header.seq, ", RosTime=", pose_pcl_msg.header.stamp.to_sec(),
              " #####")
        msg_rcvd_time = time()
        self.header.stamp = pose_pcl_msg.header.stamp
        self.pose = pose_pcl_msg.pose

        ## transformation matrices between robot and world
        self.tf_rbt_2_odom()
        self.tf_odom_2_rbt()
        self.rbt2gl_error()


        ## retrieve th, al, rds points from pointcloud
        pcl_arr = ros_numpy.point_cloud2.pointcloud2_to_array(
            pose_pcl_msg.pcl2, squeeze=True)
        pcl_arr = np.round(np.array(pcl_arr.tolist(), dtype='float'), 4)

        print("org_pcl_size: ", np.shape(pcl_arr))
        ## Downsample and assign thetas, alphas, occs, rds variables
        self.downsample_pcl(pcl_arr)

        ## define input and output data for training  
        self.gp_nav_din = np.column_stack((self.pcl_thetas, self.pcl_alphas))
        self.gp_nav_dout = np.array(self.pcl_oc, dtype='float').reshape(-1, 1)
        self.gp_nav_fit(0.09, 0.11, 0.7, 10, 10,
                        0.005)  #(ls1, ls2, var, alpha, noise, noise_var)
        nav_grd_oc, nav_grd_var = self.gp_nav.model.predict_f(self.gp_grd)
        self.gp_grd_var = nav_grd_var.numpy()
        self.gp_grd_oc = nav_grd_oc.numpy()
        self.gp_grd_rds = self.oc_srfc_rds - self.gp_grd_oc

        ## process the variance surface to define GPFrontiers and assign subgoals
        self.gp_nav_mask_thrshld()
        self.gp_grd_var_img()
        self.gp_nav_pkup_nav_pt()
        ####calculate navigation point in world frame
        ####self.gp_nav_xypts_actul_pcl()

        #### occupancy and variance surfaces visualization
        self.gp_grd_oc_pcl()
        self.gp_grd_var_pcl()

        # goal in polar coordinate wrt to velodyne
        self.gl_wrt_rbt = self.tf_2d_inv @ self.gl_wrt_odom

        # predict occupancy in direction of final goal using GP occupancy model
        self.gl_th = np.arctan2(self.gl_wrt_rbt[1], self.gl_wrt_rbt[0])
        self.gl_al = np.pi / 2
        gl_oc, gl_var = self.gp_nav.model.predict_f(
            np.array([self.gl_th, self.gl_al], dtype="float32").reshape(1, 2))
        gl_rds = self.oc_srfc_rds - gl_oc.numpy()
        # print("gl_var, var_thrshld: ", gl_var.numpy(), self.gp_nav_var_thrshld)
        # print("gl_dst_err, gl_rds: ", self.gl_dst_err, gl_rds)

        ### mode: no obstacle betwen goal and robot, robots go directly to final goal
        if (self.gl_dst_err < gl_rds):
            if not self.goal_published:
                self.glbl_gl_pbl()  ## not sure if this correct situation
                self.goal_published = False
                print("Navigation Mode: FinalGoal ")

            else:
                quit()

        ### mode: there is obstacle betwen goal and robot, robots follow recommended subgoal        
        else:
            print("Navigation Mode: SubGoal ")
            self.gp_nav_glbl_gl_pbl()

        print("Total Processing Time (with visualization): ", time() - msg_rcvd_time)

    """ @brief: error bet. robot and goal"""

    def rbt2gl_error(self):
        self.gl_dst_err = np.sqrt((self.gl_x - self.pose.x) ** 2 +
                                  (self.gl_y - self.pose.y) ** 2)
        self.gl_dir = np.arctan2(self.gl_y - self.pose.y,
                                 self.gl_x - self.pose.x)
        self.gl_dir_err = self.gl_dir - self.pose.theta
        # print("gl_dst_err, gl_dir_err: ", self.gl_dst_err, self.gl_dir_err )

    """ @brief: varinace threshoild based on the variance distribution (mean and variance)"""

    def gp_nav_mask_thrshld(self):
        gp_nav_var_stats = stats.describe(self.gp_grd_var)
        gp_nav_var_mean = gp_nav_var_stats.mean[0]
        gp_nav_var_var = gp_nav_var_stats.variance[0]
        self.gp_nav_var_thrshld = 0.6 * (gp_nav_var_mean - 3 * gp_nav_var_var)
        ##print("variance distribution mean and var: ", gp_nav_var_mean, gp_nav_var_var)
        ##print("variance threshold: ", self.gp_nav_var_thrshld)

    """ @brief: convert variance to cv image and detect high variance regions usign the variance threshold"""

    def gp_grd_var_img(self):
        """把 GP 方差网格转为二维图像，并用轮廓中心提取 frontier。

        此处的 frontier 不是全局地图的路口名称，而是“当前视野中方差高、
        可能通向未知区域”的局部球面方向。输出写入：

        * ``gp_nav_frntr_cntrs``: 每个候选的 ``[theta_rel, alpha]``；
        * ``gp_nav_frntr_areas``: 对应未知区域面积，用作 Goal Manager 权重。
        """
        img = np.zeros((self.gp_grd_h, 3 * self.gp_grd_w), np.uint8)
        ## normlize variance
        var_xtnd = np.array([]).reshape(-1, 1)
        var_xtnd = np.append(var_xtnd, self.gp_grd_var)
        var_xtnd = np.append(var_xtnd, self.gp_grd_var)
        var_xtnd = np.append(var_xtnd, self.gp_grd_var)
        # print("var_xtnd: ", np.shape(var_xtnd))
        var_norm = np.linalg.norm(var_xtnd)
        normalized_var = var_xtnd / var_norm
        bw_var = (255 / normalized_var[normalized_var.argmax(axis=0)]
                  ) * normalized_var

        ## img[0:self.gp_grd_h, 0:self.gp_grd_w] = 400*self.gp_oc_srfc_grd_var.reshape(self.gp_oc_srfc_grd_w, self.gp_oc_srfc_grd_h).T
        img[0:self.gp_grd_h,
        0:3 * self.gp_grd_w] = bw_var.reshape(3 * self.gp_grd_w, self.gp_grd_h).T

        ## grey to binay
        self.gp_nav_img_thrshld = 0.3 * int(
            np.mean(bw_var) + self.gp_nav_var_thrshld * np.var(bw_var))
        _, bw_img = cv2.threshold(img, self.gp_nav_img_thrshld, 255,
                                  cv2.THRESH_BINARY)
        ### resize image
        scale_factor = 10
        width = int(bw_img.shape[1] * scale_factor)
        height = int(bw_img.shape[0] * scale_factor)
        dsize = (width, height)
        scaled_img = cv2.resize(bw_img, dsize)
        contours, hierarchy = cv2.findContours(scaled_img.copy(),
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        l_frntr_cntrs = []
        l_frntr_ars = []
        # print(">> l_frntr_areas: ", [cv2.contourArea(i) for i in contours])
        for i in contours:
            M = cv2.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.drawContours(scaled_img, [i], -1, (0, 255, 0), 2)
                cv2.circle(scaled_img, (cx, cy), 7, (0, 0, 255), -1)

            cx = int(cx / scale_factor)
            cy = int(cy / scale_factor)
            # print(f"x: {cx} y: {cy} area:{cv2.contourArea(i)}")
            if cx >= 513 and cx < 1026 and cv2.contourArea(
                    i) > 2000 * scale_factor and cy < 13 and cy > 3:
                l_frntr_cntrs = l_frntr_cntrs + [
                    self.gp_grd_ths[cx - 513], self.gp_grd_als[cy]
                ]
                l_frntr_ars.append(cv2.contourArea(i))
                ara = l_frntr_ars[0]

        # print(">> l_frntr_cntrs: ", l_frntr_cntrs)
        # print(">> l_frntr_areas: ", l_frntr_ars)

        if (len(l_frntr_ars) == 1
            and l_frntr_ars[0] > 800000000) or len(l_frntr_ars) == 0:
            print("could not find frontier above threshold")
            # 不再兜底四个方向，直接认为无 frontier（交给上层 no_frontier 逻辑处理）
            l_frntr_cntrs = []
            l_frntr_ars = []
        self.gp_nav_frntr_cntrs = np.array(l_frntr_cntrs).reshape(-1, 2)
        self.gp_nav_frntr_areas = np.array(l_frntr_ars)  #.reshape(-1)

        ## draw contours
        if self.gp_nav_var_img_viz:
            cv2.drawContours(scaled_img, contours, -1, 255, 3, cv2.LINE_AA,
                             hierarchy, abs(-1))
            cv2.imshow('contours', scaled_img)
            cv2.waitKey()

    """ @brief: robot to wolrd 2D TF matrix"""

