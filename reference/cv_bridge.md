# 本文重要是介绍cv_bridge库的用法
因为opencv/yolo获取的图像是以python中的numpy来存储的，但是在ROS2中相机发布的是sensor_msgs信息</br>
所以cv_bridge就是构建这两个信息转换的桥梁
cv_bridge用法如下文件所示
[cv_bridge](./cv_bridge/CVmsgToROSmsg.py)
主要的作用就是实现消息类型的转换

```python
    from cv_bridge import CvBridge
    bridge=CvBridge()
    cv_image=bridge.imgmsg_to_cvmsg(image_msg,"bgr8")
```
不知道大家好奇不好奇为啥时候bgr8这个是什么用呀</br>
是因为我们的opencv的库的颜色识别的格式是bgr8，所以我们就要使用bgr格式的编码形式，这样才可以使得颜色正确传递</br>
下面是常见图像的格式</br>
opencv图像特有的格式是bgr8</br>
一般图像库是rgb8</br>
深度图像的格式是mono16</br>
灰度图像的格式是mono8</br>


    

