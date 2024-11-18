#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Set up the camera capture with the correct settings
cap = cv2.VideoCapture("/dev/video0")
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y', '1', '6', ' '))
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

# Spot meter size and position as percentages
x_center_percent, y_center_percent = 50, 50  # Center of the image
width_percent, height_percent = 5, 10       # Size of the spot meter as a percentage of frame dimensions

# Text settings
position, font, font_scale, font_color, thickness, line_type = (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA


def main():
    rospy.init_node('boson_thermal_camera_driver', anonymous=True)
    
    # ROS publishers
    scaled_pub = rospy.Publisher('/boson/scaled_image', Image, queue_size=10)
    colour_pub = rospy.Publisher('/boson/colourmap_image', Image, queue_size=10)
    roi_pub = rospy.Publisher('/boson/roi_image', Image, queue_size=10)
    zoom_pub = rospy.Publisher('/boson/zoom_image', Image, queue_size=10)

    bridge = CvBridge()

    while not rospy.is_shutdown():
        stream_ret, frame = cap.read()
        if stream_ret and frame.dtype == np.uint16:
            height, width = frame.shape[:2]
            
            # Calculate the spotmeter rectangle using percentages
            x_center = int(width * x_center_percent / 100)
            y_center = int(height * y_center_percent / 100)
            rect_width = int(width * width_percent / 100)
            rect_height = int(height * height_percent / 100)
            
            rect_top_left = (x_center - rect_width // 2, y_center - rect_height // 2)
            rect_bottom_right = (x_center + rect_width // 2, y_center + rect_height // 2)

            # Extract rectangle area and calculate temperature
            rect = frame[rect_top_left[1]:rect_bottom_right[1], rect_top_left[0]:rect_bottom_right[0]]
            mean_val = cv2.mean(rect)
            avg_pixel_val = round(mean_val[0], 1)
            avg_temp_c = round((avg_pixel_val / 100) - 273, 1)

            max_val = np.max(rect)
            max_pixel_val = round(max_val, 1)
            max_temp_c = round((max_pixel_val / 100) - 273, 1)

            min_val = np.min(rect)
            min_pixel_val = round(min_val, 1)
            min_temp_c = round((min_pixel_val / 100) - 273, 1)
            
            # Normalize the frame for display
            display_frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            img_gray = display_frame
            img_col = cv2.applyColorMap(img_gray.astype(np.uint8), cv2.COLORMAP_INFERNO)

            # Draw rectangle and temperature text on the image
            roi_image = img_col.copy()
            #text = f"ROI Average Temp C {avg_temp_c:.2f}"
            text = f"AVG {avg_temp_c:.2f}, MAX {max_temp_c:.2f}, MIN {min_temp_c:.2f}"
            cv2.rectangle(roi_image, rect_top_left, rect_bottom_right, (255, 255, 255), 2)
            cv2.putText(roi_image, text, position, font, font_scale, font_color, thickness, line_type)

            # zoom_rect = frame[rect_top_left[1]:rect_bottom_right[1], rect_top_left[0]:rect_bottom_right[0]]

            zoomed_frame = cv2.resize(rect, (640, 512), interpolation=cv2.INTER_LINEAR)
            display_zoomed_frame = cv2.normalize(zoomed_frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            col_zoom = cv2.applyColorMap(display_zoomed_frame.astype(np.uint8), cv2.COLORMAP_INFERNO)



            # Publish images to ROS topics
            
            grey_image_msg = bridge.cv2_to_imgmsg(img_gray, encoding="mono8")
            scaled_pub.publish(grey_image_msg)
            col_image_msg = bridge.cv2_to_imgmsg(img_col, encoding="bgr8")
            colour_pub.publish(col_image_msg)
            roi_image_msg = bridge.cv2_to_imgmsg(roi_image, encoding="bgr8")
            roi_pub.publish(roi_image_msg)
            zoomed_frame_msg = bridge.cv2_to_imgmsg(col_zoom, encoding="bgr8")
            zoom_pub.publish(zoomed_frame_msg)

            rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
