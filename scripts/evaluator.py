#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt

from fs_msgs.msg import Track
from nav_msgs.msg import Odometry
from fs_msgs.msg import FinishedSignal, GoSignal


class TrackEval:
    def __init__(self):
        rospy.Subscriber('/fsds/testing_only/track', Track, self.track_callback)
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.pose_callback)
        rospy.Subscriber('/fsds/signal/finished', FinishedSignal, self.finished_callback)

        self.track = []
        self.actual_pose = None
        self.finnish = False

        rospy.loginfo('Evaluator started. Waiting for GO signal.')

        go = rospy.wait_for_message('/fsds/signal/go', GoSignal)
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
        while not self.finnish:
            #plt.pause(0.01)
            track = np.array(self.track)
            plt.scatter(track[:,0], track[:,1], c='b', s=1)

            if self.actual_pose:
                plt.scatter(self.actual_pose.position.x, self.actual_pose.position.y, c='r', s=1)


if __name__ == '__main__':
    rospy.init_node('track_evaluator', anonymous=True)

    te = TrackEval()
    rospy.spin()
