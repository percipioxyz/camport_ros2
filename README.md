1.build project:
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release

2.config system env:
    source ./install/setup.bash

3.publish message:
    ros2 launch percipio_camera percipioi_camera.launch.py

4.list topics / services / parameters
    ros2 topic list
    ros2 service list
    ros2 param list

5.run rviz2:
  ros2 run rviz2 rviz2
  a.Create visualization by toptic
  b.Select Camera toptic:
        /camera/color/camera_info
        /camera/color/image_raw
        /camera/depth/camera_info
        /camera/depth/image_raw
        /camera/depth/points
        /camera/depth_registered/points
        /camera/left_ir/camera_info
        /camera/left_ir/image_raw
        /camera/right_ir/camera_info
        /camera/right_ir/image_raw
