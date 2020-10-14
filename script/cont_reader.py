#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import rospy
import rospkg
import yaml
import numpy as np
import geometry_msgs.msg

class ContinuousGraspCandid():
    def __init__(self, package = 'fgpg', path = 'sample/cont_grasp.yaml'):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path(package)
        self.file_path = self.package_path + '/' + path
        with open(self.file_path, 'r') as stream:
            self.yaml = yaml.safe_load(stream)

    def get_grasp(self, index, ratio):
        lb = np.array(self.yaml['grasp_points'][index]['lower_bound'])
        ub = np.array(self.yaml['grasp_points'][index]['upper_bound'])
        ori = np.array(self.yaml['grasp_points'][index]['orientation'])

        return ((ub-lb) * ratio + lb, ori)

    def get_grasp_pose_msg(self, index, ratio):
        g = self.get_grasp(index,ratio)
        pose_msg = geometry_msgs.msg.Pose()
        pose_msg.position.x = g[0][0]
        pose_msg.position.y = g[0][1]
        pose_msg.position.z = g[0][2]
        pose_msg.orientation.x = g[1][0]
        pose_msg.orientation.y = g[1][1]
        pose_msg.orientation.z = g[1][2]
        pose_msg.orientation.w = g[1][3]
        return pose_msg

if __name__ == '__main__':
    cgc = ContinuousGraspCandid('fgpg','script/assembly_cont.yaml')
    print(cgc.get_grasp(5,0.5))
    print(cgc.get_grasp_pose_msg(5,0.5))
    
