
rosbag record /camera/color/camera_info \
              /camera/color/image_raw \
              /camera/depth/camera_info \
              /camera/depth/color/points \
              /camera/depth/image_rect_raw \
              /camera/extrinsics/depth_to_color \
              /c16/lslidar_points \
              /vlp/velodyne_points \
              /msimu/nav/filtered_imu/data \
              /msimu/nav/heading \
              /msimu/nav/odom \
              /Odometry \
              /odometry/wheel \
              /scout_status \
              /cmd_vel \
              /tf \
              /tf_static \
              --split --duration=1m \