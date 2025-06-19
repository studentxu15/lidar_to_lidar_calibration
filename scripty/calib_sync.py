#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
import tkinter as tk
import threading
import time
import signal

class CalibrationGUI:
    def __init__(self, master):
        rospy.init_node('calib_sync', anonymous=True)

        self.master = master
        master.title("Lidar Calibration Control")
        master.protocol("WM_DELETE_WINDOW", self.on_close)  # 处理窗口关闭事件

        # 打印当前节点的完整命名空间
        node_name = rospy.get_name()
        full_param_path = rospy.resolve_name('~flag_topic')
        # rospy.loginfo(f"当前节点名称: {node_name}")
        # rospy.loginfo(f"完整参数路径: {full_param_path}")

        self.topic_name = rospy.get_param('~flag_topic', 'calib_trig')
        # rospy.loginfo(f"最终使用的topic_name: {self.topic_name}")
        self.pub = rospy.Publisher(self.topic_name, Header, queue_size=10)
        
        # 设置窗口大小和位置
        master.geometry("400x300+500+300")
        
        # 创建标签
        self.label = tk.Label(master, text="选择要发送的frame_id:")
        self.label.pack(pady=20)
        
        # 创建按钮
        self.btn_frame1 = tk.Button(master, text="开始标定", command=lambda: self.send_calibration_flag("calibration_start"))
        self.btn_frame1.pack(pady=10, ipadx=20, ipady=10)
        
        self.btn_frame2 = tk.Button(master, text="记录点位", command=lambda: self.send_calibration_flag("calibration_record_points"))
        self.btn_frame2.pack(pady=10, ipadx=20, ipady=10)
        
        self.btn_frame3 = tk.Button(master, text="计算外参", command=lambda: self.send_calibration_flag("calibration_computer"))
        self.btn_frame3.pack(pady=10, ipadx=20, ipady=10)
        
        # 启动ROS后台线程
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True  # 设置为守护线程
        self.ros_thread.start()
    
    def ros_spin(self):
        """ROS后台运行"""
        while not rospy.is_shutdown():
            time.sleep(0.1)
    
    def on_close(self):
        """处理窗口关闭事件"""
        rospy.signal_shutdown("GUI closed")  # 关闭ROS节点
        self.master.destroy()  # 关闭窗口
        
    def send_calibration_flag(self, frame_id):
        if hasattr(self, 'pub'):  # 确保pub已初始化
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = frame_id
            self.pub.publish(header)
            rospy.loginfo(f"已发送frame_id: {frame_id}")

def signal_handler(sig, frame):
    rospy.signal_shutdown("SIGINT received")
    root.quit()

def periodic_check():
    if rospy.is_shutdown():
        root.quit()
    else:
        root.after(100, periodic_check)

def main():
    global root
    root = tk.Tk()
    app = CalibrationGUI(root)
    signal.signal(signal.SIGINT, signal_handler)
    root.after(100, periodic_check)
    root.mainloop()

if __name__ == '__main__':
    main()