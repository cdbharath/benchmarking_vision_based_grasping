#!/usr/bin/env python3

# Python program to illustrate 
# saving an operated video
  
# organize imports
import cv2
import rospy
import rospkg
import datetime
import os

def list_ports():
    """
    Test the ports and returns a tuple with the available ports and the ones that are working.
    """
    non_working_ports = []
    dev_port = 0
    working_ports = []
    available_ports = []
    while len(non_working_ports) < 6: # if there are more than 5 non working ports stop the testing. 
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            non_working_ports.append(dev_port)
            print("Port %s is not working." %dev_port)
        else:
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                working_ports.append(dev_port)
            else:
                print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                available_ports.append(dev_port)
        dev_port +=1

    return working_ports[0]

class VideoRecoder:
    def __init__(self, camera_name=None):
        rp = rospkg.RosPack()

        if camera_name is None:
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
        ret, frame = self.cap.read() 
        if not ret:
            rospy.logerr("[VideoRecorder] No frame received")
        else:
            # output the frame
            self.out.write(frame) 

    def on_shutdown(self):
        # Close the window / Release webcam
        self.cap.release()
        
        # After we release our webcam, we also release the output
        self.out.release() 

if __name__ == "__main__":
    rospy.init_node("video_recorder")
    port = list_ports()
    video_recorder = VideoRecoder(port)
    rospy.spin()