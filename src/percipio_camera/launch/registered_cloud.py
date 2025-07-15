#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import threading
import sys
import os
import time
import struct
from datetime import datetime

def get_current_zoom(view_control):
    """获取当前视角的缩放级别"""
    if hasattr(view_control, 'get_zoom'):
        return view_control.get_zoom()
    else:
        # 旧版本替代方法
        camera_params = view_control.convert_to_pinhole_camera_parameters()
        camera_pos = np.linalg.inv(camera_params.extrinsic)[:3, 3]
        return np.linalg.norm(camera_pos)

class PointCloudViewer3D(Node):
    def __init__(self):
        super().__init__('pointcloud_3d_viewer')
        
        # 声明参数
        self.declare_parameter('point_size', 1.0)
        self.declare_parameter('save_path', os.path.expanduser('./pointcloud_saves'))
        self.declare_parameter('pointcloud_topic', '/camera/depth_registered/points')
        
        # 获取参数值
        self.point_size = self.get_parameter('point_size').value
        self.save_path = self.get_parameter('save_path').value
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        
        # 点云可视化参数
        self.background_color = [0.1, 0.1, 0.1]
        self.latest_cloud = None
        self.update_view = False
        self.lock = threading.Lock()  # 线程锁保护共享数据
        
        # 创建保存目录
        os.makedirs(self.save_path, exist_ok=True)
        self.get_logger().info(f"点云将保存至: {self.save_path}")
        
        # 设置PointCloud2订阅器
        self.sub = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            10  # 队列大小
        )
        self.get_logger().info(f"订阅点云话题: {pointcloud_topic}")
        
        # 启动Open3D可视化线程
        self.vis_thread = threading.Thread(target=self.visualization_thread, daemon=True)
        self.vis_thread.start()
        
        # 打印使用说明
        self.get_logger().info("3D PointCloud Viewer Ready")

    def pointcloud_callback(self, msg):
        """PointCloud2消息回调函数"""
        try:
            # 从PointCloud2消息中提取点和颜色
            points = []
            colors = []

            # 获取点云字段信息以确定颜色格式
            field_names = [field.name for field in msg.fields]
            has_rgb = 'rgb' in field_names

            if has_rgb:
                # 处理单独的r,g,b字段
                read_fields = ['x', 'y', 'z', 'rgb']
            else:
                # 没有颜色信息
                read_fields = ['x', 'y', 'z']

            # 使用pc2.read_points解析点云
            gen = pc2.read_points(
                msg, 
                field_names=read_fields,
                skip_nans=True
            )
            
            # 获取字段数据类型信息
            field_types = {}
            for field in msg.fields:
                field_types[field.name] = field.datatype
            
            for p in gen:
                if(p[2] < 10) :
                    points.append([p[0], p[1], p[2]])
                
                    # 处理颜色信息
                    if has_rgb:
                        # 处理合并的rgb/rgba字段
                        color_val = p[3]
                        # 处理不同类型的颜色表示
                        if isinstance(color_val, float):
                            # 将浮点颜色转换为整数
                            color_val_arr = struct.pack('<f', color_val) 
                            color_val = color_val_arr[0] + (color_val_arr[1] << 8) | (color_val_arr[2] << 16)
                        
                        # 转换32位整数到RGB
                        r = ((color_val >> 16) & 0x0000ff) / 255.0
                        g = ((color_val >> 8) & 0x0000ff) / 255.0
                        b = (color_val & 0x0000ff) / 255.0
                        colors.append([r, g, b])
                    else:
                        # 没有颜色信息时使用白色
                        colors.append([1.0, 1.0, 1.0])
            
            if not points:
                self.get_logger().warn("收到空点云消息!")
                return
            
            # 创建Open3D点云
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(points))
            
            # 添加颜色信息
            pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
            
            # 使用线程锁保护共享数据
            with self.lock:
                self.latest_cloud = pcd
                self.update_view = True
            
        except Exception as e:
            self.get_logger().error(f"处理点云错误: {str(e)}")
            _, _, exc_tb = sys.exc_info()
            self.get_logger().error(f"错误位置: 第 {exc_tb.tb_lineno} 行")

    def save_current_pointcloud(self, cloud):
        """保存当前点云到文件"""
        if cloud is None or len(cloud.points) == 0:
            self.get_logger().warn("没有点云数据可保存!")
            return False
            
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"registered_cloud_{timestamp}.ply"
        filepath = os.path.join(self.save_path, filename)
        
        try:
            # 保存点云
            o3d.io.write_point_cloud(filepath, cloud)
            self.get_logger().info(f"点云已保存至: {filepath}")
            self.get_logger().info(f"点数: {len(cloud.points)}")
            return True
        except Exception as e:
            self.get_logger().error(f"保存点云失败: {str(e)}")
            return False

    def visualization_thread(self):
        """Open3D可视化线程"""
        # 延迟初始化解决OpenGL上下文问题
        time.sleep(1.0)
        
        # 创建可视化窗口
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name='ROS2 PointCloud2 Viewer',
            width=1280,
            height=720,
            visible=True
        )
        
        # 初始空点云
        dummy_cloud = o3d.geometry.PointCloud()
        dummy_cloud.points = o3d.utility.Vector3dVector(np.array([[0,0,0]]))
        vis.add_geometry(dummy_cloud)
        ctr = vis.get_view_control()
        
        # 设置渲染选项
        render_opt = vis.get_render_option()
        render_opt.background_color = np.array(self.background_color)
        render_opt.point_size = self.point_size
        render_opt.light_on = True
        
        current_view_params = None
        first_frame = True
        
        while rclpy.ok():
            # 检查更新标志
            cloud_to_update = None
            with self.lock:
                if self.update_view and self.latest_cloud is not None:
                    cloud_to_update = self.latest_cloud
                    self.update_view = False
            
            # 更新点云显示
            if cloud_to_update is not None:
                # 保存当前视图状态（如果已有）
                if current_view_params:
                    prev_view_params = ctr.convert_to_pinhole_camera_parameters()
                
                # 更新点云
                vis.clear_geometries()
                vis.add_geometry(cloud_to_update)
                
                # 智能设置初始视角
                if first_frame:
                    center = np.array([0,   0,   0.5])
                    
                    eye_angle2 = center + np.array([0, 0, 5])      # 正上方
                    lookat2 = center
                    up2 = np.array([0, 1, 0])                      # Y轴向上

                    ctr.set_lookat(lookat2)
                    ctr.set_up(up2)
                    ctr.set_front(eye_angle2 - lookat2)
                    ctr.set_zoom(0.5)  # 适合室内场景的缩放值
                    
                    self.save_current_pointcloud(self.latest_cloud)
                    first_frame = False
                elif prev_view_params:
                    # 恢复之前的视图状态
                    ctr.convert_from_pinhole_camera_parameters(prev_view_params)
            
            # 更新渲染
            vis.poll_events()
            vis.update_renderer()

            # 保存当前视图状态（用于下一次更新）
            current_view_params = ctr.convert_to_pinhole_camera_parameters()
            
            time.sleep(0.05)
        
        # 关闭窗口
        vis.destroy_window()
        self.get_logger().info("Open3D可视化已关闭")
        rclpy.try_shutdown()

def main(args=None):
    # 解决Open3D在无显示环境中的问题
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
        print(f"设置DISPLAY环境变量为: {os.environ['DISPLAY']}")
    
    rclpy.init(args=args)
    viewer = PointCloudViewer3D()
    
    try:
        # 在独立线程中运行rclpy.spin
        spin_thread = threading.Thread(target=rclpy.spin, args=(viewer,), daemon=True)
        spin_thread.start()
        
        # 等待可视化线程结束
        viewer.vis_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        print("节点已关闭")

if __name__ == '__main__':
    main()