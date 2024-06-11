1.build project:
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release / colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

2.config system env:
    source ./install/setup.bash

3.publish message:
    ros2 launch percipio_camera percipioi_camera.launch.py

4.run rviz2:
  ros2 run rviz2 rviz2
  a.Create visualization by toptic
  b.Select Camera toptic(
        /color
            /image_raw/Image
        /depth
            /image_raw/Image
            /points/PointCloud2
        /depth_registered
            /points/PointCloud2
        /left_ir
            /image_raw/Image
        /right_ir
            /image_raw/Image
