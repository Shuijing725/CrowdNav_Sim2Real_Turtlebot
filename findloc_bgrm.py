#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml
import PIL.Image as Img
from numpy import asarray
import pickle
import cv2
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan, Image
import matplotlib.pyplot as plt
import PIL.Image as pil_img
from cv_bridge import CvBridge
from sklearn.neighbors import KDTree
import sys

try:
    filename = sys.argv[1]
    if len(sys.argv) > 2:
        raise Exception("Enter only one filename!")
except:
    raise Exception("Invalid argument! Enter a valid filename. e.g. csl_entrance")

f = open(filename+'.yaml')
current_map = yaml.load(f)
img = Img.open(filename+'.pgm')
map_width, map_height = img.width, img.height
map_resolution = current_map['resolution']
map_origin_x = current_map['origin'][0]
map_origin_y = current_map['origin'][1]

# img_np = asarray(img)
# print(img_np.shape)
# map_1d = img_np.reshape(map_height*map_width)
# print(map_1d)
# a=[]
# for i in range(len(map_1d)):
#     if map_1d[i] not in a:
#         a.append(map_1d[i])
# print(a)

dyn_pub = rospy.Publisher('person_pts', LaserScan, queue_size=1)
map_pub = rospy.Publisher('compare_map', Image, queue_size=1)

class Dynamic_Map:

    def __init__(self):

        filename = 'csl_entrance'

        f = open(filename + '.yaml')
        current_map = yaml.load(f)
        img = pil_img.open(filename+'.pgm')
        map_width, map_height = img.width, img.height
        map_resolution = current_map['resolution']
        self.origin_x= current_map['origin'][0]
        self.origin_y = current_map['origin'][1]
        print('width:', map_width, 'height:', map_height)
        print('origin:', self.origin_x, self.origin_y)
        print("resolution",map_resolution)

        assert self.origin_x < 0 and self.origin_y < 0, "change axis range in map plots for debugging"

        # detection parameters
        self.prox_thre = 0.25 #meters
        self.max_range = 25.0 #meters

        # initialize the robot state
        self.rx     = 0.0
        self.ry     = 0.0
        self.rtheta = 0.0
        self.loc_ready = False

        print("waiting for localization...")

        # read the static map
        # static_map = cv2.imread('csl_entrance.pgm', 0)
        static_map = cv2.imread(filename+'.pgm', 0)
        static_pts = np.array(np.where(static_map == 0)).T
        static_pts[:, [1, 0]] = static_pts[:, [0, 1]]
        static_pts[:, 0] = self.origin_x / map_resolution + static_pts[:, 0] # x coordinates
        static_pts[:, 1] = map_height + self.origin_y / map_resolution - static_pts[:, 1] # y coordinates

        static_pts = static_pts * map_resolution

        # construct the k-d tree
        self.tree = KDTree(static_pts)

        # plot maps for debugging
        self.debug      = True
        self.figsize    = (8, 8)
        self.bridge     = CvBridge()
        self.static_pts = static_pts

    def update(self, msg):

        if self.loc_ready:

            # read lidar parameters
            resolution = msg.angle_increment
            angles = np.arange(0, 2*np.pi, resolution)

            # under robot coordinate
            ranges = np.clip(msg.ranges, 0.0, self.max_range)

            points_x = np.cos(angles) * ranges
            points_y = np.sin(angles) * ranges

            data = np.concatenate((points_x[:, np.newaxis], points_y[:, np.newaxis]), axis=1)

            # under world coordinate
            R = np.array([[np.cos(self.rtheta), - np.sin(self.rtheta)],
                          [np.sin(self.rtheta), np.cos(self.rtheta)]])
            t = np.array([[self.rx],
                          [self.ry]])
            data = (np.dot(R, data.T) + t).T

            # remove background points
            dist, ind = self.tree.query(data, k=1)

            ranges[np.where(dist.flatten() <= self.prox_thre)] = np.inf

            msg.ranges = list(ranges)
            dyn_pub.publish(msg)

            if self.debug:
                # plot maps for debugging
                fig = plt.figure(figsize=self.figsize)
                plt.axes([0,0,1,1])
                plt.axis([self.origin_x, -self.origin_x, self.origin_y, -self.origin_y])
                plt.axis('off')
                plt.plot(self.static_pts[:, 0], self.static_pts[:, 1],
                         marker='.', markersize=0.8, linestyle='None', color='black')
                plt.plot(data[:, 0], data[:, 1],
                         marker='.', markersize=0.8, linestyle='None', color='red')
                fig.canvas.draw()
                compare_map = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
                compare_map = compare_map.reshape(fig.canvas.get_width_height()[::-1] + (3,))
                compare_map = cv2.cvtColor(compare_map, cv2.COLOR_RGB2BGR)
                compare_map_msg = self.bridge.cv2_to_imgmsg(compare_map)
                map_pub.publish(compare_map_msg)
                plt.close()

    def robot_pose_update(self, data):

        self.rx, self.ry, self.rtheta = data

        if not self.loc_ready:
            print("robot localized!")
            self.loc_ready = True


if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            # print(trans)
            # calculate x,y of base_link in the map frame
            x_, y_ = trans.transform.translation.x, trans.transform.translation.y
            x_r, y_r, z_r, w_r = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
            
            (roll,pitch,yaw) = euler_from_quaternion([x_r,y_r,z_r,w_r])
            # print([x_, y_,yaw])
            coord_in_map_slam_origin_x = map_origin_x + map_width/2
            coord_in_map_slam_origin_y = map_origin_y + map_height/2
            coord_in_map_x = int( x_ / map_resolution + coord_in_map_slam_origin_x)
            coord_in_map_y = int(y_ / map_resolution + coord_in_map_slam_origin_y)
            
            pub = rospy.Publisher('robot_pose', Float32MultiArray, queue_size=1)
            data_to_send = Float32MultiArray()
            data_to_send.data = [x_, y_, yaw]
            pub.publish(data_to_send)

            dynamic_map = Dynamic_Map()
            rospy.Subscriber('scan', LaserScan, dynamic_map.update, queue_size=1)
            dynamic_map.robot_pose_update(data=[x_, y_, yaw])
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            print('no')
            continue
        rate.sleep()

