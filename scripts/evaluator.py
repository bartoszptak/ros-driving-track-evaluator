#!/usr/bin/env python3
import rospy
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pyquaternion import Quaternion
from scipy.ndimage.interpolation import rotate
import scipy.interpolate as interp

from fs_msgs.msg import Track, ExtraInfo
from nav_msgs.msg import Odometry
from fs_msgs.msg import FinishedSignal, GoSignal

class TrackEval:
    def __init__(self):
        self.track = []
        self.actual_pose = None
        self.finnish = False
        self.visited = None
        self.last_state = True
        self.penelaties = []
        self.hit_cones = 0

        self.tolerance = 1.0+float(rospy.get_param('~tolerance'))/100
        self.visualize = bool(rospy.get_param('~visualize'))

        self.width = float(rospy.get_param('~car_width'))
        self.length = float(rospy.get_param('~car_length'))

        self.finish_line = [4.0, 0.0]

        rospy.Subscriber(rospy.get_param('~track_topic'), Track, self.track_callback)
        rospy.Subscriber(rospy.get_param('~odom_topic'), Odometry, self.pose_callback)
        rospy.Subscriber(rospy.get_param('~finished_signal'), FinishedSignal, self.finished_callback)
        rospy.Subscriber('/fsds/testing_only/extra_info', ExtraInfo, self.hit_cones_callback)

        while self.visited is None:
            rospy.sleep(0.1)

        rospy.loginfo('Evaluator started. Waiting for GO signal.')

        go = rospy.wait_for_message(rospy.get_param('~go_signal'), GoSignal)
        rospy.loginfo('GO signal received.')
        self.total_time = time.time()
        self._main_func()

    def hit_cones_callback(self, data):
        self.hit_cones = data.doo_counter

    def track_callback(self, data: Track):
        while self.actual_pose is None:
            rospy.sleep(0.01)

        for cone in data.track:           
            self.track.append([cone.location.x, cone.location.y, cone.color])

        self.track = np.array(self.track)
        self.blue = self.track[self.track[:, 2]==0][:,:2]
        self.yellow = self.track[self.track[:, 2]==1][:,:2]

        self.track_width = (self.yellow-self.blue)/2

        mask = self.yellow[:, 0]>self.blue[:, 0]

        self.blue[:, 0][mask] -= self.track_width[:, 0][mask]
        self.yellow[:, 0][mask] -= self.track_width[:, 0][mask]

        self.blue[:, 0][~mask] += self.track_width[:, 0][~mask]
        self.yellow[:, 0][~mask] += self.track_width[:, 0][~mask]

        self.centers = self.blue+self.track_width
        new_centers = np.empty((self.centers.shape[0]*3,2))
        new_centers[::3] = self.centers
        new_centers[1:-2:3] = self.centers[:-1] + (self.centers[1:]-self.centers[:-1])/3
        new_centers[2:-2:3] = self.centers[:-1] + (self.centers[1:]-self.centers[:-1])*2/3
        new_centers[-2] = self.centers[-1] + (self.centers[0]-self.centers[-1])/3
        new_centers[-1] = self.centers[-1] + (self.centers[0]-self.centers[-1])*2/3
        self.centers = new_centers

        new_track_width = np.empty((self.track_width.shape[0]*3,2))
        new_track_width[::3] = self.track_width
        new_track_width[1:-2:3] = self.track_width[:-1] + (self.track_width[1:]-self.track_width[:-1])/3
        new_track_width[2:-2:3] = self.track_width[:-1] + (self.track_width[1:]-self.track_width[:-1])*2/3
        new_track_width[-2] = self.track_width[-1] + (self.track_width[0]-self.track_width[-1])/3
        new_track_width[-1] = self.track_width[-1] + (self.track_width[0]-self.track_width[-1])*2/3
        self.track_width = new_track_width

        self.visited = np.zeros((self.centers.shape[0]))

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
        val = np.apply_along_axis(lambda x: (np.linalg.norm(x[:2]-car_points, axis=1)<=self.tolerance*np.hypot(*x[2:])).all(), axis=1, arr=np.concatenate([self.centers, self.track_width], axis=1))
        self.visited[val]= True

        val = val.any()

        if val and not self.last_state:
            self.penelaties[-1] = time.time()-self.penelaties[-1]
            print(f'[LOGS] Back to road! Penelaties: {self.penelaties[-1]}s')
        elif not val and self.last_state:
            self.penelaties.append(time.time())
            print('[LOGS] Out of road!')

        self.last_state = val


    def pose_callback(self, data: Odometry):
        self.actual_pose = data.pose.pose


    def finished_callback(self, data: FinishedSignal):
        if data:
            self.finish = True
            if not self.last_state:
                self.penelaties[-1] = time.time()-self.penelaties[-1]


            track_time = time.time() - self.total_time
            rospy.loginfo('FINNISH signal received.')
            print('*'*40)

            print('* Time of run: '+str(np.round(track_time, 3))+'s')
            print('* Out of road penelaties: '+ str(np.round(np.sum(self.penelaties), 3))+'s'+ ' ('+str(len(self.penelaties)) + ' x)')
            print('* Hitted cones penelaties: '+ str(self.hit_cones*2.0)+'s'+ ' ('+str(self.hit_cones) + ' x)')
            print('* Percent of road: ' + str(np.round(np.sum(self.visited)/self.visited.shape[0]*100, 2)) + '%')
            print('*')
            print('* Total: '+ str(np.round(track_time+np.sum(self.penelaties)+self.hit_cones*2.0, 3)) +'s')
            print('*'*40)
            rospy.signal_shutdown(0)

    def _main_func(self):

        if self.visualize:        
            fig, ax = plt.subplots(1, 1)
            plt.ion()
        while not self.finnish:

            if self.visualize:
                plt.pause(0.01)
                ax.clear()

                for x in np.concatenate([self.centers, self.track_width], axis=1):
                    circle = plt.Circle((x[0], x[1]), self.tolerance*np.hypot(*x[2:]), color='blue', fill=False)
                    ax.add_patch(circle)

                ax.scatter(self.blue[:,0], self.blue[:,1], c='b', s=1)
                ax.scatter(self.yellow[:,0], self.yellow[:,1], c='gold', s=1)
                ax.scatter(self.centers[:,0], self.centers[:,1], c='red', s=1)

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

                if (self.finish_line[0]-2 < self.actual_pose.position.x < self.finish_line[0]-1) and (self.finish_line[1]-10 < self.actual_pose.position.y < self.finish_line[1]+10):
                    if (np.sum(self.visited)/self.visited.shape[0]) > 0.05:
                        m = FinishedSignal()
                        self.finished_callback(m)
                        break

                if self.visualize:
                    ax.scatter(car_points[:, 0], car_points[:, 1], c='g', s=1)


if __name__ == '__main__':
    rospy.init_node('track_evaluator', anonymous=True)

    te = TrackEval()
    rospy.spin()
