#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vanishing point detector (ROS node)
- 输入: /kinect/hd/image_color_rect
- 输出: /vanish_point (geometry_msgs/Pose)
  * position.x = 估算的前方距离 (m)，基于消失点像素 + 相机高度/仰角
  * position.y = 0
  * position.z 未使用

思路:
1) 灰度 + Sobel 边缘 + Otsu 二值
2) 自定义 Hough 变换找直线 (rho, theta)
3) 只保留近竖直的线 (5~85 度或 95~175 度)
4) 选角度差最大的两条线求交点 -> 消失点
5) 用相机模型(内参+高度+仰角)估算前方距离
"""

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

# 相机参数（需按实际标定调节）
FX = 640.5098521801531
FY = 640.5098521801531
CX = 640.5
CY = 360.5
CAM_HEIGHT = 0.6  # 相机安装高度 (m)
CAM_TILT_DEG = 0.5  # 相机仰角 (deg)


def rgb2gray(img: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)


def sobel_filter(gray_input: np.ndarray) -> np.ndarray:
    # 用较低精度并用 numpy 自行归一化，避免某些平台上 OpenCV 优化指令触发非法指令
    gx = cv2.Sobel(gray_input, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray_input, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = np.sqrt(gx * gx + gy * gy, dtype=np.float32)
    m = float(magnitude.max())
    if m > 1e-6:
        norm = (magnitude / m) * 255.0
    else:
        norm = magnitude
    return norm.astype(np.uint8)


def hough_transform(binary_img: np.ndarray):
    thetas = np.deg2rad(np.arange(0, 180.0, 0.5))
    diag_len = np.ceil(np.sqrt(binary_img.shape[0] ** 2 + binary_img.shape[1] ** 2))
    rhos = np.linspace(-diag_len, diag_len, int(2 * diag_len))
    accumulator = np.zeros((len(rhos), len(thetas)), dtype=np.int32)
    y_idxs, x_idxs = np.nonzero(binary_img)

    cos_t = np.cos(thetas)
    sin_t = np.sin(thetas)
    rho_values = (x_idxs[:, None] * cos_t + y_idxs[:, None] * sin_t).astype(int)
    diag_len_2 = 2 * diag_len
    rho_indices = np.round((rho_values + diag_len) / diag_len_2 * len(rhos)).astype(int)
    np.add.at(accumulator, (rho_indices, np.arange(len(thetas))), 1)
    return accumulator, thetas, rhos


def get_vanishing_point(pair_index, rho_arr, theta_arr, diffs, pairs):
    """根据角度差排名 pair_index 的两条线求交点"""
    diff_sorted = sorted(enumerate(diffs), key=lambda x: x[1], reverse=True)
    max_idx = diff_sorted[pair_index][0]
    theta1_deg, theta2_deg = pairs[max_idx]
    theta1_idx = np.where(theta_arr == theta1_deg)
    theta2_idx = np.where(theta_arr == theta2_deg)
    theta1 = np.deg2rad(theta1_deg)
    theta2 = np.deg2rad(theta2_deg)
    rho1 = rho_arr[theta1_idx]
    rho2 = rho_arr[theta2_idx]

    denom = (np.cos(theta1) * np.sin(theta2) - np.sin(theta1) * np.cos(theta2))
    if np.abs(denom) < 1e-6:
        return None
    x = int((rho1 * np.sin(theta2) - rho2 * np.sin(theta1)) / denom)
    y = int((rho2 * np.cos(theta1) - rho1 * np.cos(theta2)) / denom)
    return x, y


def pixel_to_distance(x_pix: int, y_pix: int) -> float:
    # 像素到归一化坐标
    x = (x_pix - CX) / FX
    y = (y_pix - CY) / FY
    angle = np.arctan2(y, 1.0) + np.radians(CAM_TILT_DEG)
    if np.tan(angle) == 0:
        return float("inf")
    return CAM_HEIGHT / np.tan(angle)


class VanishPointNode:
    def __init__(self):
        self.pub = rospy.Publisher("/vanish_point", Pose, queue_size=1)
        rospy.Subscriber("/kinect/hd/image_color_rect", Image, self.on_image, queue_size=1)
        self.pose_msg = Pose()

    def on_image(self, msg: Image):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "vanish_point: failed to parse image: %s", exc)
            return

        gray = rgb2gray(img)
        edge = sobel_filter(gray)
        _, binary = cv2.threshold(edge, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        acc, thetas, rhos = hough_transform(binary.astype(np.uint8))
        acc[acc < 175] = 0
        rows, cols = np.nonzero(acc)
        if len(rows) < 2:
            self.pose_msg.position.x = 0.0
            self.pose_msg.position.y = 0.0
            self.pub.publish(self.pose_msg)
            return

        # 映射到 (rho, theta)
        rho_list = [rhos[r] for r in rows]
        theta_list = [np.rad2deg(thetas[c]) for c in cols]

        # 去重：角度差 < 10 的只保留一个
        theta_filtered, rho_filtered = [], []
        for r, t in zip(rho_list, theta_list):
            if all(abs(t - tf) >= 10 for tf in theta_filtered):
                theta_filtered.append(t)
                rho_filtered.append(r)

        # 只保留近竖直的线
        theta_valid = []
        rho_valid = []
        for r, t in zip(rho_filtered, theta_filtered):
            if (5 <= t <= 85) or (95 <= t <= 175):
                theta_valid.append(t)
                rho_valid.append(r)

        theta_valid = np.array(theta_valid)
        rho_valid = np.array(rho_valid)
        if len(theta_valid) < 2:
            self.pose_msg.position.x = 0.0
            self.pose_msg.position.y = 0.0
            self.pub.publish(self.pose_msg)
            return

        # 所有角度对的差值
        diffs = []
        pairs = []
        for i in range(len(theta_valid)):
            for j in range(i + 1, len(theta_valid)):
                diffs.append(abs(theta_valid[i] - theta_valid[j]))
                pairs.append((theta_valid[i], theta_valid[j]))
        if len(diffs) == 0:
            self.pose_msg.position.x = 0.0
            self.pose_msg.position.y = 0.0
            self.pub.publish(self.pose_msg)
            return

        # 依次尝试角度差最大的几对，找落在图像内的交点
        vp = None
        for k in range(min(3, len(diffs))):
            pt = get_vanishing_point(k, rho_valid, theta_valid, diffs, pairs)
            if pt is None:
                continue
            x, y = pt
            if 0 <= x < msg.width and 0 <= y < msg.height:
                vp = (x, y)
                break
            # 如果都在外面，最终也用第一个
            if k == 0:
                vp = (x, y)

        if vp is None:
            self.pose_msg.position.x = 0.0
            self.pose_msg.position.y = 0.0
            self.pub.publish(self.pose_msg)
            return

        dist = abs(pixel_to_distance(vp[0], vp[1]))
        self.pose_msg.position.x = float(dist)
        self.pose_msg.position.y = 0.0
        self.pose_msg.position.z = 0.0
        self.pub.publish(self.pose_msg)


def main():
    rospy.init_node("vanish_point_node")
    VanishPointNode()
    rospy.loginfo("vanish_point_node started, subscribing /kinect/hd/image_color_rect")
    rospy.spin()


if __name__ == "__main__":
    main()
