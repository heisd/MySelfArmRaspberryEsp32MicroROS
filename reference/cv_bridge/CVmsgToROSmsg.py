# 这个程序运行没有意义，但是已经很明显的显示了cv_bridge的用法
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# 定义
cv_image=cv2.imread("./picture")
# 定义一个桥对象
bridge=CvBridge()
# ROS和opencv图像的互相转换
# OpenCV  -> ROS image
ros_image_msg=bridge.cv2_to_imgmsg(cv_image,"bgr8")
# ROS image -> OpenCV  
cv_image=bridge.imgmsg_to_cv2(ros_image_msg,"bgr8")
class YoloDetector:
    def __init__(self):
        self.bridge=CvBridge()
    # 获取过信息进行处理,这里的msg指的是从摄像头传递给ROS2Node的图像
    def image_callback(self,msg):
         # 一般工作流程如下,ROS2通过消息机制获得图像->转换为OPencv的图像
        cv_image=self.bridge.imgmsg_to_cv2(msg,"bgr8")
        # 使用Yolo处理
        results=self.model(cv_model)
        # 将图像处理好的结果反馈给ROS2
        result_msg =self.bridge.cv2_to_imgmsg(cv_image,"bgr8")
if __name__ == "__main__":
    YoloDetector

   

