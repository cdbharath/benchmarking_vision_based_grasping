#!/usr/bin/env python3

# Python program to illustrate 
# saving an operated video
  
# organize imports
import cv2
import rospy
import rospkg
import datetime
import os
  
class VideoRecoder:
    def __init__(self):
        rp = rospkg.RosPack()

        camera_name = rospy.get_param("camera_name")
        output_file_name = datetime.datetime.now().strftime("%I:%M%p on %B %d, %Y") + ".mp4"
        output_file_dir = rp.get_path("benchmarking_grasp")
        output_file_path = os.path.join(output_file_dir, "recordings", output_file_name)
        fps = 30

        # This will return video from the first webcam on your computer.
        self.cap = cv2.VideoCapture(camera_name)  

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(output_file_path, fourcc, fps, (640, 480))
  
        rospy.Timer(rospy.Duration(1/fps), self.timer_cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("[Video Recorder] Node initialized")

    def timer_cb(self, event):
        # reads frames from a camera 
        # ret checks return at each frame
        _, frame = self.cap.read() 
            
        # output the frame
        self.out.write(frame) 

    def on_shutdown(self):
        # Close the window / Release webcam
        self.cap.release()
        
        # After we release our webcam, we also release the output
        self.out.release() 

if __name__ == "__main__":
    rospy.init_node("video_recorder")
    video_recorder = VideoRecoder()
    rospy.spin()