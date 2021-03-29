#!/usr/bin/env python3
import rospy
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pyquaternion import Quaternion
from scipy.ndimage.interpolation import rotate
import scipy.interpolate as interp
import xmltodict

from nav_msgs.msg import Odometry
from av_msgs.msg import Mode 


class TrackEval:
    def __init__(self):
        with open(rospy.get_param('~world_path'), 'r') as f:
            world = xmltodict.parse(f.read())

        points = [p.split(' ')[:2] for p in world['sdf']['world']['road']['point']]
        self.track_width = float(world['sdf']['world']['road']['width'])/2

        self.points = np.array(points, dtype=np.float32)

        x = np.abs(self.points[:1]-self.points[1:])
        n_points = np.sqrt(x[:,0]**2 + x[:, 1]**2)/10
        n_points[n_points<10] = 10

        x_l = np.abs(self.points[-2]-self.points[0])
        n_l = np.sqrt(x_l[0]**2 + x_l[1]**2)/10

        self.points = np.concatenate(
            [np.linspace(self.points[i], self.points[i+1], int(n_points[i]), endpoint=False) for i in range(n_points.shape[0])]+\
            [np.linspace(self.points[-2], self.points[0], 10 if n_l < 10 else n_l, endpoint=False)], axis=0
        )

    
        rospy.Subscriber(rospy.get_param('~odom_topic'), Odometry, self.pose_callback)
        rospy.Subscriber(rospy.get_param('~go_signal'), Mode, self.mode_callback)
        self.actual_pose = None

        self.visualize = bool(rospy.get_param('~visualize'))
        self.width = float(rospy.get_param('~car_width'))
        self.length = float(rospy.get_param('~car_length'))
        self.tolerance = 1.0+float(rospy.get_param('~tolerance'))/100

        self.last_state = True       
        self.is_start = False 
        self.penelaties = []

        rospy.loginfo('Evaluator started. Waiting for GO signal.')

        while not self.is_start:
            rospy.sleep(0.1)

        rospy.loginfo('GO signal received.')
        self._main_func()

    @staticmethod
    def rotate(p, origin=(0, 0), degrees=0):
        angle = np.deg2rad(degrees)
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
        o = np.atleast_2d(origin)
        p = np.atleast_2d(p)
        return np.squeeze((R @ (p.T-o.T) + o.T).T)

    def _get_car_model_points(self, x, y, angle):
        c = np.array([x,y])

        points = np.concatenate([
            np.linspace(c+np.array([-self.length/2, -self.width/2]), c+np.array([self.length/2, -self.width/2]), 10),
            np.linspace(c+np.array([self.length/2, -self.width/2]), c+np.array([self.length/2, self.width/2]), 10),
            np.linspace(c+np.array([self.length/2, self.width/2]), c+np.array([-self.length/2, self.width/2]), 10),
            np.linspace(c+np.array([-self.length/2, self.width/2]), c+np.array([-self.length/2, -self.width/2]), 10),
        ], axis=0)

        points = self.rotate(points, origin=c, degrees=angle)

        return points

    def check_in_track(self, car_points: np.ndarray):
        val = np.apply_along_axis(lambda x: (np.linalg.norm(x[:2]-car_points, axis=1)<=self.tolerance*self.track_width).all(), axis=1, arr=self.points).any()

        if val and not self.last_state:
            self.penelaties[-1] = time.time()-self.penelaties[-1]
            print(f'[LOGS] Back to road! Penelaties: {self.penelaties[-1]}s')
        elif not val and self.last_state:
            self.penelaties.append(time.time())
            print('[LOGS] Out of road!')

        self.last_state = val

    def pose_callback(self, data: Odometry):
        self.actual_pose = data.pose.pose

    def mode_callback(self, data: Mode):
        if data:
            if data.selfdriving:
                self.is_start = True
            else:
                self.finish = True
                if not self.last_state:
                    self.penelaties[-1] = time.time()-self.penelaties[-1]

                rospy.loginfo('FINNISH signal received.')
                print(f'Penelaties {np.sum(self.penelaties)}')
                rospy.signal_shutdown(0)

    def _main_func(self):

        if self.visualize:  
            fig, ax = plt.subplots(1, 1)
            plt.ion()

        while True:
            if self.visualize:  
                ax.clear()

                for x in self.points:
                    circle = plt.Circle((x[0], x[1]), self.track_width, color='blue', fill=False)
                    ax.add_patch(circle)

                ax.scatter(self.points[:,0], self.points[:,1], c='b', s=1)

            if self.actual_pose:

                q = Quaternion(
                    self.actual_pose.orientation.w, 
                    self.actual_pose.orientation.x, 
                    self.actual_pose.orientation.y, 
                    self.actual_pose.orientation.z,
                    axis=[0,0,1])
                theta = np.degrees(q.yaw_pitch_roll[0])+180

                car_points = self._get_car_model_points(self.actual_pose.position.x, self.actual_pose.position.y, theta)
                self.check_in_track(car_points)

                if self.visualize:  
                    ax.scatter(car_points[:, 0], car_points[:, 1], c='red', s=5)

            if self.visualize:  
                plt.pause(0.01)


if __name__ == '__main__':
    rospy.init_node('track_evaluator', anonymous=True)

    te = TrackEval()
    rospy.spin()
