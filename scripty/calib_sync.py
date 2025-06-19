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
        master.protocol("WM_DELETE_WINDOW", self.on_close)

        self.topic_name = rospy.get_param('~flag_topic', 'calib_trig')
        self.pub = rospy.Publisher(self.topic_name, Header, queue_size=10)
        self.sub = rospy.Subscriber(self.topic_name, Header, self.feedback_callback)

        master.geometry("400x350+500+300")  # 增加点高度留位置给新控件

        self.current_count = 0

        self.label = tk.Label(master, text=f"当前记录点数: {self.current_count}")
        self.label.pack(pady=10)

        self.btn_frame1 = tk.Button(master, text="开始标定", command=lambda: self.send_calibration_flag("calibration_start"))
        self.btn_frame1.pack(pady=5, ipadx=20, ipady=10)

        self.btn_frame2 = tk.Button(master, text="记录点位", command=lambda: self.send_calibration_flag("calibration_record_points"))
        self.btn_frame2.pack(pady=5, ipadx=20, ipady=10)

        self.btn_frame3 = tk.Button(master, text="计算外参", command=lambda: self.send_calibration_flag("calibration_computer"))
        self.btn_frame3.pack(pady=5, ipadx=20, ipady=10)

        # 新增输入框和按钮
        self.erase_frame = tk.Frame(master)
        self.erase_frame.pack(pady=15)

        self.erase_label = tk.Label(self.erase_frame, text="擦除点序号:")
        self.erase_label.pack(side=tk.LEFT)

        self.erase_entry = tk.Entry(self.erase_frame, width=10)
        self.erase_entry.pack(side=tk.LEFT, padx=5)

        self.erase_btn = tk.Button(self.erase_frame, text="发送擦除请求", command=self.send_erase_request)
        self.erase_btn.pack(side=tk.LEFT, padx=5)

        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()

    def ros_spin(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)

    def on_close(self):
        rospy.signal_shutdown("GUI closed")
        self.master.destroy()

    def send_calibration_flag(self, frame_id):
        if hasattr(self, 'pub'):
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = frame_id
            self.pub.publish(header)
            rospy.loginfo(f"已发送frame_id: {frame_id}")

    def feedback_callback(self, msg):
        if msg.frame_id == "feedback_number":
            new_count = msg.seq
            self.master.after(0, self.update_count_label, new_count)

    def update_count_label(self, count):
        self.current_count = count
        self.label.config(text=f"当前记录点数: {self.current_count}")

    def send_erase_request(self):
        # 读取输入框内容
        seq_str = self.erase_entry.get()
        try:
            seq_num = int(seq_str)
            if seq_num < 0:
                raise ValueError("序号不能为负数")
        except ValueError:
            rospy.logwarn("请输入有效的非负整数序号")
            return

        if hasattr(self, 'pub'):
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "request_erase"
            header.seq = seq_num
            self.pub.publish(header)
            rospy.loginfo(f"已发送擦除请求，seq={seq_num}")

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
