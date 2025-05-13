#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class WheelVelocityCalculator:
    def __init__(self):
        # Tham số robot
        self.wheel_diameter = 0.05  # m
        self.wheel_separation = 0.25  # m
        self.ticks_per_rev = 2000  # xung mỗi vòng

        # Publisher và Subscriber
        self.vel_pub = rospy.Publisher('/wheel_velocities', Twist, queue_size=10)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        # Lấy vận tốc góc của bánh trước
        try:
            left_idx = msg.name.index('joint_before_left')
            right_idx = msg.name.index('joint_before_right')
            omega_l = msg.velocity[left_idx]  # rad/s
            omega_r = msg.velocity[right_idx]  # rad/s
        except ValueError:
            rospy.logwarn("Không tìm thấy joint_befor_left hoặc joint_befor_right")
            return

        # Tính tốc độ tuyến tính của bánh
        v_l = omega_l * (self.wheel_diameter / 2.0)
        v_r = omega_r * (self.wheel_diameter / 2.0)

        # Tính tốc độ tuyến tính và góc của robot
        linear_vel = (v_l + v_r) / 2.0
        angular_vel = (v_r - v_l) / self.wheel_separation

        # Tính số xung mỗi giây
        ticks_l = omega_l * (self.ticks_per_rev / (2.0 * 3.14159))
        ticks_r = omega_r * (self.ticks_per_rev / (2.0 * 3.14159))

        # Xuất bản dữ liệu
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.vel_pub.publish(twist_msg)

        rospy.loginfo(f"Ticks Left: {ticks_l:.2f}, Ticks Right: {ticks_r:.2f}, Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f}")

if __name__ == '__main__':
    rospy.init_node('wheel_velocity_calculator', anonymous=True)
    calc = WheelVelocityCalculator()
    rospy.spin()