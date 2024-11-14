#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


cap = cv2.VideoCapture("/dev/video0")
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

# Spot meter size, position
x_center, y_center, width, height = 320, 256, 100, 100
rect_top_left = (int(x_center - width / 2), int(y_center - height / 2))
rect_bottom_right = (int(x_center + width / 2), int(y_center + height / 2))
position, font, font_scale, font_color, thickness, line_type = (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA


def main():
    rospy.init_node('boson_thermal_camera_driver', anonymous=True)
    
    scaled_pub = rospy.Publisher('/boson/scaled_image', Image, queue_size=10)
    colour_pub = rospy.Publisher('/boson/colourmap_image', Image, queue_size=10)
    roi_pub = rospy.Publisher('/boson/roi_image', Image, queue_size=10)

    bridge = CvBridge()

   
    while not rospy.is_shutdown():
        stream_ret, frame = cap.read()
        if stream_ret and frame.dtype == np.uint16:
            # Extract rectangle area and calculate temperature
            rect = frame[rect_top_left[1]:rect_bottom_right[1], rect_top_left[0]:rect_bottom_right[0]]
            mean_val = cv2.mean(rect)
            #avg_pixel_val = round(mean_val[0], 1)
            avg_temp_c = round((avg_pixel_val / 100) - 273, 1)
            #avg_temp_f = round(avg_temp_c * 9/5 + 32, 1)

            
            display_frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            img_gray = display_frame
            img_col = cv2.applyColorMap(img_gray.astype(np.uint8), cv2.COLORMAP_INFERNO)

            roi_image = img_col.copy()
            text = f"ROI Average Temp C {avg_temp_c:.2f} "
            cv2.rectangle(roi_image, rect_top_left, rect_bottom_right, (255, 255, 255), 2)
            cv2.putText(roi_image, text, position, font, font_scale, font_color, thickness, line_type)

            roi_image = bridge.cv2_to_imgmsg(roi_image, encoding="bgr8")
            roi_pub.publish(roi_image)
            grey_image = bridge.cv2_to_imgmsg(img_gray, encoding="mono8")
            scaled_pub.publish(grey_image)
            col_image = bridge.cv2_to_imgmsg(img_col, encoding="bgr8")
            colour_pub.publish(col_image)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
