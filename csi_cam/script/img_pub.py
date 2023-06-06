#!/usr/bin/env python
#coding=utf-8

import numpy as np
import os, json
import rospy, rospkg
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from swarm_msgs.msg import BoundingBox, BoundingBoxes


# 设置gstreamer管道参数
def gstreamer_pipeline(
    capture_width=1280, #摄像头预捕获的图像宽度
    capture_height=720, #摄像头预捕获的图像高度
    display_width=1280, #窗口显示的图像宽度
    display_height=720, #窗口显示的图像高度
    framerate=60,       #捕获帧率
    flip_method=0,      #是否旋转图像
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )



if __name__ == '__main__':
    src_path = os.path.join(rospkg.RosPack().get_path("offboard_pkg"), "..")
    setting_file = open(os.path.join(src_path, "settings.json"))
    setting = json.load(setting_file)
    print(json.dumps(setting, indent=4))

    is_show_rgb = setting["Debug"]["is_show_rgb"]
    is_show_hsv = setting["Debug"]["is_show_hsv"]
    print("rgb:",is_show_rgb)
    print("hsv:",is_show_hsv)
    
    low_range1 = np.array([169, 80, 80])   #[0, 230, 80]
    high_range1 = np.array([181, 256, 256]) # [5, 256, 200]

    low_range2 = np.array([0, 120, 80])   #[0, 230, 80]
    high_range2 = np.array([5, 256, 256]) # [5, 256, 200]

    capture_width = 1280
    capture_height = 720

    display_width = 1280
    display_height = 720

    framerate = 30			# 帧数
    flip_method = 0			# 方向

    # 创建管道
    print(gstreamer_pipeline(capture_width,capture_height,display_width,display_height,framerate,flip_method))

    #管道与视频流绑定
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)


    rospy.init_node('iris_fpv_cam', anonymous=True)
    bridge = CvBridge()
    imag_pub = rospy.Publisher("tracker/pos_image", BoundingBoxes, queue_size=10)  # 发送图像位置
    image_raw_pub = rospy.Publisher("image_raw", Image, queue_size=1)  # 发送图像位置

    img_pos = BoundingBoxes()
    img_pos.header.stamp = rospy.Time.now()


    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)

        # 逐帧显示
        while cv2.getWindowProperty("CSI Camera", 0) >= 0 and not rospy.is_shutdown():
            ret_val, cv_img = cap.read()

            # image_raw_pub.publish(bridge.cv2_to_imgmsg(cv_img))

            hue_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            th1 = cv2.inRange(hue_image, low_range1, high_range1)
            th2 = cv2.inRange(hue_image, low_range2, high_range2)
            th = th1 + th2
            dilated = cv2.dilate(th, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)

            # 连通域分析, https://stackoverflow.com/questions/35854197/how-to-use-opencvs-connected-components-with-stats-in-python/35854198#35854198
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dilated)
            id = 0
            for i in range(1, num_labels):          # 0是背景
                left = stats[i, cv2.CC_STAT_LEFT]
                top  = stats[i, cv2.CC_STAT_TOP]
                w = stats[i, cv2.CC_STAT_WIDTH]
                h = stats[i, cv2.CC_STAT_HEIGHT]
                area = stats[i, cv2.CC_STAT_AREA]

                if area > 16:
                    bbox = BoundingBox()
                    bbox.probability = 1
                    bbox.xmin = left
                    bbox.ymin = top
                    bbox.xmax = left+w
                    bbox.ymax = top+h
                    bbox.id = id
                    img_pos.bounding_boxes.append(bbox)
                    id += 1

                    if is_show_rgb:
                        cv2.rectangle(cv_img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 255, 0), 2)

            # 是否显示图像
            if is_show_rgb:
                cv2.imshow("img", cv_img)
                cv2.waitKey(1)
            if is_show_hsv:
                cv2.imshow("hsv", dilated)
                cv2.waitKey(1)

            # 按横坐标排序
            id = 0
            img_pos.bounding_boxes.sort(key=lambda x: (x.xmin+x.xmax)/2)
            for bbox in img_pos.bounding_boxes:
                bbox.id = id
                id += 1
            
            imag_pub.publish(img_pos)

        cap.release()
        cv2.destroyAllWindows()
    else:
        print("打开摄像头失败") 

