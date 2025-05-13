#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def lidar_callback(data):
    # In ra số lượng mẫu quét
    rospy.loginfo("Number of range measurements: %d", len(data.ranges))
    # In ra khoảng cách tại một số góc (ví dụ: phía trước, 0 độ)
    mid_index = len(data.ranges) // 2  # Góc 0 độ (phía trước)
    rospy.loginfo("Distance at 0 degrees: %.2f meters", data.ranges[mid_index])
    # In ra khoảng cách tối thiểu
    
    valid_ranges = [r for r in data.ranges if r > data.range_min and r < data.range_max]
    if valid_ranges:  # Kiểm tra danh sách không rỗng
        min_distance = min(valid_ranges)
        rospy.loginfo("Minimum distance: %.2f meters", min_distance)
    else:
        rospy.loginfo("No valid range measurements within range_min and range_max")

def lidar_reader():
    # Khởi tạo node
    rospy.init_node('lidar_reader', anonymous=True)
    # Subscribe vào topic /scan
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    # Giữ node chạy
    rospy.spin()

if __name__ == '__main__':
    try:
        lidar_reader()
    except rospy.ROSInterruptException:
        pass