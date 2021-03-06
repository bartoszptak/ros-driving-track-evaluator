#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pyquaternion import Quaternion

from fs_msgs.msg import Track
from nav_msgs.msg import Odometry
from fs_msgs.msg import FinishedSignal, GoSignal

class TrackEval:
    def __init__(self):
        rospy.Subscriber(rospy.get_param('~track_topic'), Track, self.track_callback)
        rospy.Subscriber(rospy.get_param('~odom_topic'), Odometry, self.pose_callback)
        rospy.Subscriber(rospy.get_param('~finished_signal'), FinishedSignal, self.finished_callback)

        self.track = []
        self.actual_pose = None
        self.finnish = False

        self.width = float(rospy.get_param('~car_width'))
        self.length = float(rospy.get_param('~car_length'))

        rospy.loginfo('Evaluator started. Waiting for GO signal.')

        go = rospy.wait_for_message(rospy.get_param('~go_signal'), GoSignal)
        rospy.loginfo('GO signal received.')
        self._main_func()

    def track_callback(self, data: Track):
        for cone in data.track:
            self.track.append([cone.location.x, cone.location.y])

    def pose_callback(self, data: Odometry):
        self.actual_pose = data.pose.pose


    def finished_callback(self, data: FinishedSignal):
        if data:
            self.finish = True
            rospy.loginfo('FINNISH signal received.')


    def _main_func(self):
        fig, ax = plt.subplots()

        while not self.finnish:
            plt.pause(0.01)
            track = np.array(self.track)
            ax.scatter(track[:,0], track[:,1], c='b', s=1)

            if self.actual_pose:

                q = Quaternion(
                    self.actual_pose.orientation.w, 
                    self.actual_pose.orientation.x, 
                    self.actual_pose.orientation.y, 
                    self.actual_pose.orientation.z,
                    axis=[0,0,1])
                theta = np.degrees(q.yaw_pitch_roll[0])
                
                rect = patches.Rectangle(
                    (self.actual_pose.position.x-self.length/2, self.actual_pose.position.y-self.width/2), 
                    self.length, self.width, angle=theta, 
                    fill=True, edgecolor='r', facecolor='r')
                ax.add_patch(rect)


if __name__ == '__main__':
    rospy.init_node('track_evaluator', anonymous=True)

    te = TrackEval()
    rospy.spin()
