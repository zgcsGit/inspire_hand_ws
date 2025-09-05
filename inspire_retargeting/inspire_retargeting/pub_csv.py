#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from handpoints_publisher.msg import HandKeypoints

import numpy as np
from pathlib import Path
from typing import List

from dex_retargeting.constants import RobotName, RetargetingType, HandType, get_default_config_path
from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting.seq_retarget import SeqRetargeting
from inspire_retargeting.single_hand_detector import SingleHandDetector


class RetargetSubscriber(Node):
    """
    实时订阅 /Lefthandpoints，执行 retargeting 并发布 /joint_states
    """
    def __init__(self):
        super().__init__("retarget_subscriber")

        # 发布 JointState
        self.publisher_ = self.create_publisher(JointState, "/joint_states", 10)

        # 初始化 SingleHandDetector
        self.hand_detector = SingleHandDetector(hand_type="Left", selfie=False)

        # 初始化 Retargeting
        robot_name = RobotName.inspire
        retargeting_type = RetargetingType.dexpilot
        hand_type = HandType.left

        # 配置文件路径
        config_path = get_default_config_path(robot_name, retargeting_type, hand_type)

        # 如果 dex-retargeting 是 pip 包，手动指定 assets 路径
        # 这里请改为你实际 pip 安装 dex-retargeting 后的 assets 路径，或者放一份在 ROS 包里
        robot_dir = Path("/home/zz/inspire_hand_ws/src/dex-retargeting/assets/robots/hands")
        if not robot_dir.exists():
            self.get_logger().error(f"URDF dir {robot_dir} not exists. 请确认路径！")
            raise FileNotFoundError(robot_dir)
        RetargetingConfig.set_default_urdf_dir(str(robot_dir))

        self.retargeting: SeqRetargeting = RetargetingConfig.load_from_file(config_path).build()
        self.joint_names: List[str] = self.retargeting.optimizer.robot.dof_joint_names

        # 订阅 HandKeypoints 消息
        self.subscription = self.create_subscription(
            HandKeypoints,
            "/Lefthandpoints",
            self.handpoints_callback,
            10
        )
        self.get_logger().info("RetargetSubscriber 初始化完成，等待 HandKeypoints 消息...")

    def handpoints_callback(self, msg: HandKeypoints):
        """
        回调函数：接收到 HandKeypoints 消息后执行 retargeting 并发布 JointState
        """
        if not msg.points or len(msg.points) != 21:
            self.get_logger().warn("收到的 HandKeypoints 点数不正确，跳过。")
            return

        # geometry_msgs/Point 转 np.array
        frame_pts = np.array([[p.x, p.y, p.z] for p in msg.points])

        # 使用 SingleHandDetector 处理
        num_box, joint_pos, _, wrist_rot = self.hand_detector.detect_from_array(frame_pts)
        if num_box == 0 or joint_pos is None:
            self.get_logger().warn("SingleHandDetector 未检测到有效手，跳过。")
            return

        # 构造 ref_value
        retargeting_type = self.retargeting.optimizer.retargeting_type
        indices = self.retargeting.optimizer.target_link_human_indices
        if retargeting_type == "POSITION":
            ref_value = joint_pos[indices, :]
        else:
            origin_indices = indices[0, :]
            task_indices = indices[1, :]
            ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]

        # 调用 retarget
        qpos = self.retargeting.retarget(ref_value)

        # 发布 JointState
        msg_out = JointState()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.name = self.joint_names
        msg_out.position = qpos.tolist()
        self.publisher_.publish(msg_out)


def main():
    rclpy.init()
    node = RetargetSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
