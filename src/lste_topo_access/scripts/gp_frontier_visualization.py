"""One focused responsibility extracted from the legacy GP frontier node."""

import numpy as np
import ros_numpy
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

class GpFrontierVisualizationMixin:
    def glbl_gl_pbl(self):
        # yaw = np.arctan2(self.gl_x, self.gl_y)
        # qtrn = quaternion_from_euler(0, 0, yaw)
        rbt_gl_msg = PoseStamped()
        rbt_gl_msg.pose.position.x = np.cos(self.pose.theta) * (self.gl_x - self.pose.x) + \
                                     np.sin(self.pose.theta) * (self.gl_y - self.pose.y)
        rbt_gl_msg.pose.position.y = - np.sin(self.pose.theta) * (self.gl_x - self.pose.x) + \
                                     np.cos(self.pose.theta) * (self.gl_y - self.pose.y)
        self.header.frame_id = "os_sensor"
        rbt_gl_msg.header = self.header
        self.gp_subgl_pub.publish(rbt_gl_msg)
        print("recomended subgoal is global goal, in os_sensor frame: ", rbt_gl_msg.pose.position.x,
              rbt_gl_msg.pose.position.y)
        # print("glbl_nav goal: ", nav_xy_gl[0], nav_xy_gl[1])

    """ @brief:  stop command when robot reach final goal"""

    def stop_cmd(self):
        gl_msg = PoseStamped()
        gl_msg.pose.position.x = self.pose.x
        gl_msg.pose.position.y = self.pose.y
        gl_msg.pose.position.z = 0
        gl_msg.pose.orientation.x = 0
        gl_msg.pose.orientation.y = 0
        gl_msg.pose.orientation.z = 0
        gl_msg.pose.orientation.w = 1
        self.header.frame_id = "odom"
        gl_msg.header = self.header
        self.gp_rcmndd_subgl_pub.publish(gl_msg)

    """ @brief:  publish variance surface for visualization"""

    def gp_grd_var_pcl(self):
        rds = self.gp_nav_var_viz * np.ones(np.shape(self.gp_grd_var)[0],
                                            dtype='float32').reshape(-1, 1)
        x, y, z = self.convert_spherical_2_cartesian(
            self.gp_grd.T[:][0].reshape(-1, 1),
            self.gp_grd.T[:][1].reshape(-1, 1), rds)
        intensity = np.array(self.gp_grd_var,
                             dtype='float32').reshape(-1, 1)
        gp_var_pcl = np.column_stack((x, y, z, intensity))
        self.header.frame_id = self.gp_nav_frame_id
        pc2 = point_cloud2.create_cloud(self.header, self.fields, gp_var_pcl)
        self.gp_var_pub.publish(pc2)

    """ @brief:  publish occupancy surface for visualization"""

    def gp_grd_oc_pcl(self):
        rds = self.gp_nav_var_viz * np.ones(np.shape(self.gp_grd_rds)[0],
                                            dtype='float32').reshape(-1, 1)
        x, y, z = self.convert_spherical_2_cartesian(
            self.gp_grd.T[:][0].reshape(-1, 1),
            self.gp_grd.T[:][1].reshape(-1, 1), rds)
        intensity = np.array(self.gp_grd_rds,
                             dtype='float32').reshape(-1, 1)
        gp_oc_pcl = np.column_stack((x, y, z, intensity))
        self.header.frame_id = self.gp_nav_frame_id
        pc2 = point_cloud2.create_cloud(self.header, self.fields, gp_oc_pcl)
        self.gp_oc_pub.publish(pc2)

    """ @brief:  publish navigation points for visualization"""

    def gp_nav_pts_pcl(self):
        rds = self.gp_nav_goal_dst * np.ones(self.gp_nav_gls_sz,
                                             dtype='float32').reshape(-1, 1)
        if self.gp_nav_pts is None or self.gp_nav_pts.size == 0:
            return
        x, y, z = self.convert_spherical_2_cartesian(
            self.gp_nav_pts.T[0].reshape(-1, 1),
            self.gp_nav_pts.T[1].reshape(-1, 1), rds)
        intensity = np.array(self.gap_utlty_fun,
                             dtype='float32').reshape(-1, 1)
        nav_pts_pcl = np.column_stack((x, y, z, intensity))
        self.header.frame_id = self.gp_nav_frame_id
        pc2 = point_cloud2.create_cloud(self.header, self.fields, nav_pts_pcl)
        self.gp_nav_pts_pub.publish(pc2)

    """ @brief:  publish navigation points in odom frame for visualization"""

    def gp_nav_xypts_actul_pcl(self):
        # print(">> gp_nav_xypts_actul_pcl:: ")
        if self.gp_nav_actul_xy_gls is None or len(self.gp_nav_actul_xy_gls) == 0:
            return
        intensity = np.array(self.gap_utlty_fun,
                             dtype='float32').reshape(-1, 1)
        if intensity.shape[0] != self.gp_nav_actul_xy_gls.shape[0]:
            intensity = np.zeros((self.gp_nav_actul_xy_gls.shape[0], 1), dtype='float32')
        nav_pts_pcl = np.column_stack((self.gp_nav_actul_xy_gls, intensity))
        self.header.frame_id = "odom"
        pc2 = point_cloud2.create_cloud(self.header, self.fields, nav_pts_pcl)
        self.gp_actul_xy_subgls_pub.publish(pc2)

    #zcy
    def clear_points(self):
        # 创建一个空的 PointCloud2 消息
        self.header.frame_id = "odom"
        empty_pcl = point_cloud2.create_cloud(self.header, self.fields, [])

        # 发布空的 PointCloud2 消息以清空 gp_nav_pts_pub
        self.gp_nav_pts_pub.publish(empty_pcl)

        # 发布空的 PointCloud2 消息以清空 gp_actul_xy_subgls_pub
        self.gp_actul_xy_subgls_pub.publish(empty_pcl)

    """ @brief:  downsample pointcloud"""

    def downsample_pcl(self, pcl_arr):
        """按方位角下采样球面点云，并生成 GP 训练样本。

        输入点格式来自 oc_srfc_proj：x=theta、y=alpha、z=distance、
        intensity=占据相关量。``pcl_skp`` 越大，训练点越少、计算越快，
        但方向分辨率越低。
        """
        pcl_arr = pcl_arr[np.argsort(pcl_arr[:, 0])]  ## sort based on thetas
        thetas = pcl_arr.transpose()[:][0].reshape(-1, 1)
        self.org_unq_thetas = np.array(sorted(set(
            thetas.flatten())))  #.reshape(-1,1)
        #### percentage to keep  or fraction to delete
        keep_th_ids = [
            t for t in range(0, np.shape(self.org_unq_thetas)[0], self.pcl_skp)]
        ids = []
        for t in keep_th_ids:
            ids = ids + list(np.where(thetas == self.org_unq_thetas[t])[0])
        pcl_arr = pcl_arr[ids]
        pcl_arr = pcl_arr.transpose()
        self.pcl_thetas = np.round(pcl_arr[:][0].reshape(-1, 1), 4)
        self.pcl_alphas = np.round(pcl_arr[:][1].reshape(-1, 1), 4)
        self.pcl_rds = np.round(pcl_arr[:][2].reshape(-1, 1), 4)
        self.pcl_oc = np.round(pcl_arr[:][3].reshape(-1, 1), 4)
        self.pcl_sz = np.shape(self.pcl_thetas)[0]
        self.pcl_unq_thetas = np.array(sorted(set(
            self.pcl_thetas.flatten())))  #.reshape(-1,1)
        self.unq_smpld_th_size = np.shape(self.pcl_unq_thetas)[0]

    """ @brief:  grid to reconstruct variance surface"""

    def sample_gp_nav_grid(self):
        th_rsltion = 0.01227  #0.00174  # 0.02 # #from -pi to pi rad -> 0 35999
        al_rsltion = 0.0349  #0.0349  #vpl16 resoltuion is 2 deg (from -15 to 15 deg)
        self.gp_grd_ths = np.arange(-np.pi + 0.0, np.pi,
                                    th_rsltion, dtype='float32')
        self.gp_grd_als = np.arange(np.pi / 2 - 0.261799, np.pi / 2 + 0.261799,
                                    al_rsltion, dtype='float32')
        self.gp_grd = np.array(np.meshgrid(self.gp_grd_ths,
                                           self.gp_grd_als)).T.reshape(-1, 2)
        self.gp_grd_w = np.shape(self.gp_grd_ths)[0]
        self.gp_grd_h = np.shape(self.gp_grd_als)[0]
        # print("\ngrid: ", np.shape(self.gp_grd))
        # print("grid w, h: ", self.gp_grd_w, self.gp_grd_h)

    """ @brief:  convert spherical to cartesian coordinates"""

    def convert_spherical_2_cartesian(self, theta, alpha, dist):
        x = np.array(dist * np.sin(alpha) * np.cos(theta),
                     dtype='float32').reshape(-1, 1)
        y = np.array(dist * np.sin(alpha) * np.sin(theta),
                     dtype='float32').reshape(-1, 1)
        z = np.array(dist * np.cos(alpha), dtype='float32').reshape(-1, 1)
        return x, y, z

    """ @brief:  convert  cartesian  to spherical coordinates"""

    def convert_cartesian_2_spherical(self, x, y, z):
        dist = np.sqrt(x ** 2 + y ** 2 + z ** 2)
        theta = np.arctan2(y, x)
        alpha = np.arccos(z / dist)
        return theta, alpha, dist

    """ @brief:  callback of sph pointcloud """

    def sph_pcl_cb(self, sph_pcl_msg):
        """缓存最新球面点云；不在回调中执行昂贵的 GP 训练。"""
        # 防御：若 header 尚未初始化（极早期回调），先建一个
        if not hasattr(self, "header") or self.header is None:
            self.header = Header()
            self.header.frame_id = "odom"
        self.header.stamp = sph_pcl_msg.header.stamp
        pcl_arr = ros_numpy.point_cloud2.pointcloud2_to_array(
            sph_pcl_msg, squeeze=True)
        self.pcl_arr = np.round(np.array(pcl_arr.tolist(), dtype='float'), 4)

        """ @brief:  callback of robot pose """

    def pose_cb(self, rbt_pose_msg):
        """缓存最新二维位姿，供下一轮 step() 使用。"""
        self.pose = rbt_pose_msg

        """ @brief:  main process function """
