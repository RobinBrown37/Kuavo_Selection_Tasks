#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg  import PointStamped 
from std_msgs.msg  import Float64MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import kalman as kalm

class YOLOv8Node:
    def __init__(self):
        rospy.init_node('yolov8_node', anonymous=True)
        self.bridge = CvBridge()

        self.KF = kalm.KalmanFilter(0.1, 1, 1, 1, 0.1, 0.1)
        self.x, self.y, self.xc, self.yc = 0, 0, 0, 0

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/yolo/detections', Image, queue_size=2)
        self.box_pub  = rospy.Publisher('/yolo/box_coordinates', Float64MultiArray, queue_size=2) 
        self.center_pub  = rospy.Publisher('/yolo/center_coordinates', PointStamped, queue_size=2) 
        rate = rospy.Rate(5)
        
        # 加载 YOLOv8 模型（替换为你的模型路径）
        self.model = YOLO("/home/robinbrown/yolov8/best23_last.pt")
        print("加载完毕")

    def image_callback(self, data):
        try:
            print("接受到图像，开始处理")
            # 将 ROS Image 转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 使用 YOLOv8 进行推理
            results = self.model(cv_image)
            # print(results)
            # 在图像上绘制检测结果
            annotated_image = results[0].plot()


            # 计算中心点坐标 
            centers = [] 
            boxes = [] 
            for r in results: 
                box = r.boxes.xyxy.cpu().numpy()
                if len(box) > 0:
                    conf = r.boxes.conf.cpu().numpy()
                    max_conf_idx = np.argmax(conf)
                    [x1, y1, x2, y2] = box[max_conf_idx]
                    boxes.append([x1, y1, x2, y2])
                    centers.append(np.array([[(x1  + x2) / 2], [(y1 + y2) / 2]])) 
                    print(f"中心坐标计算完毕：", str(np.array([[(x1  + x2) / 2], [(y1 + y2) / 2]])))
                    self.x = (x1  + x2) / 2
                    self.y = (y1 + y2) / 2
                else:
                    print("没有检测到目标")
            if (len(box) > 0):
                # 卡尔曼滤波更新 
                (self.xc, self.yc) = self.KF.filter(centers[0])
            else:
                # 卡尔曼滤波预测
                self.x, self.y = self.KF.predict()
                self.xc, self.yc = self.x, self.y
                # self.x, self.y = self.xc,  self.yc
            cv2.circle(annotated_image, (int(self.x), int(self.y)), radius=5, color=(0, 0, 255), thickness=-1)

            # 发布识别框及其中心点坐标
            center_msg = PointStamped() 
            center_msg.header.stamp  = rospy.Time.now()  
            center_msg.point.x  = self.x
            center_msg.point.y  = self.y
            center_msg.point.z  = 0 
            self.center_pub.publish(center_msg)
        
            box_msg = Float64MultiArray() 
            if (len(centers)>0): 
                box_msg.data  = boxes[-1]
            else: 
                box_msg.data  = [0, 0, 0, 0] 
            self.box_pub.publish(box_msg) 
            # 发布带检测结果的图像
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image,"bgr8"))
            print("图像识别处理完毕")

        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

if __name__ == '__main__':
    try:
        yolo_node = YOLOv8Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass