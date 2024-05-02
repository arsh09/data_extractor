#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import cv2
import os
import csv
import numpy as np


class ImagePoseSaver:
    def __init__(self):
        rospy.init_node('image_pose_saver')

        self.image_sub = message_filters.Subscriber('/front/zed_node/rgb/image_rect_color/compressed', CompressedImage)
        self.depth_sub = message_filters.Subscriber('/depth_republish/compressedDepth', CompressedImage)
        self.pose_sub = rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)

        self.robot_pose = None

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], 
            queue_size=10, slop=0.1, allow_headerless=True
        )

        self.save_dir = "/data/bachus_dataset/dataset_june_2022/" #os.path.join(os.path.expanduser('~'), 'ros_data')
        if not os.path.exists(self.save_dir): 
            os.mkdir( self.save_dir )

        for i in ["rgb", "depth" ]:
            if not os.path.exists( os.path.join( self.save_dir, i ) ): 
                os.mkdir( os.path.join( self.save_dir, i ))

        cv2.namedWindow("color")
        cv2.namedWindow("depth")

        ts.registerCallback(self.callback)

        self.image_count = 0
        self.pose_data = []

    def convert_depth_image(self, msg):

        # 'msg' as type CompressedImage
        depth_fmt, compr_type = msg.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        if compr_type != "compressedDepth":
            raise Exception("Compression type is not 'compressedDepth'."
                            "You probably subscribed to the wrong topic.")

        # remove header from raw data
        depth_header_size = 12
        raw_data = msg.data[depth_header_size:]

        depth_img_raw = cv2.imdecode(np.fromstring(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
        if depth_img_raw is None:
            # probably wrong header size
            raise Exception("Could not decode compressed depth image."
                            "You may need to change 'depth_header_size'!")

        return depth_img_raw

    def callback(self, image_msg, depth_msg):
        if self.robot_pose == None: 
            return 
        
        try:
            # Convert compressed image data to OpenCV format
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Convert compressed depth data to OpenCV format
            depth_image = self.convert_depth_image( depth_msg ) 

            # # Save color image
            color_image_filename = os.path.join( self.save_dir, f'rgb/color_image_{self.image_count}.png' )
            cv2.imwrite(color_image_filename, color_image)

            # Save depth image    
            depth_image_filename = os.path.join( self.save_dir, f'depth/depth_image_{self.image_count}.png' )
            cv2.imwrite(depth_image_filename, depth_image)

            # Save pose data
            pose_msg = self.robot_pose
            self.pose_data.append([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z])

            self.image_count += 1

            cv2.imshow("color", cv2.resize(color_image, (640, 480)))
            cv2.imshow("depth", cv2.resize(depth_image, (640, 480)))
            cv2.waitKey(1)

            rospy.loginfo(f"Count: {self.image_count}")

        except Exception as e:
            rospy.logerr(f"Error processing images and poses: {str(e)}")

    def robot_pose_cb(self, pose_msg): 
        self.robot_pose = pose_msg

    def save_pose_data(self):
        csv_path = os.path.join( self.save_dir, "poses.csv" )
        with open(csv_path, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
            writer.writerows(self.pose_data)

    def run(self):
        
        rospy.spin()
        self.save_pose_data()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        saver = ImagePoseSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
