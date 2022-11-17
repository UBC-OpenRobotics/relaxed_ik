#! /usr/bin/env python
'''
author: Yousif El-Wishahy
website: https://yel-wishahy.github.io/ 
email: yel.wishahy@gmail.com
'''
######################################################################################################

import rospy
from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_ik.msg import EEPoseGoals, JointAngles
from relaxed_ik.srv import RelaxedIKService, RelaxedIKServiceRequest, RelaxedIKServiceResponse
from std_msgs.msg import Float32
from RelaxedIK.Utils.colors import bcolors

class RelaxedIKServiceServer:
    relaxedIK:RelaxedIK = None

    def __init__(self):
        rospy.init_node('relaxed_ik_service')
        rospy.loginfo("==== Loading Relaxed IK Solver ====")
        config_file_name = rospy.get_param('config_file_name', default='relaxedIK.config')
        RelaxedIKServiceServer.relaxedIK = RelaxedIK.init_from_config(config_file_name)

        rospy.loginfo("==== Loading Relaxed IK Service ====")
        rospy.Service('relaxed_ik_service', RelaxedIKService, RelaxedIKServiceServer.handle_relaxedik_request)
        rospy.loginfo("==== Ready for ik point requests ====")

        rospy.spin()

    @staticmethod
    def handle_relaxedik_request(req):
        relaxedIK = RelaxedIKServiceServer.relaxedIK
        num_chains = RelaxedIKServiceServer.relaxedIK.vars.robot.numChains
        if req.pose_goals is None or relaxedIK is None or num_chains == 0:
            return RelaxedIKServiceResponse()
        eepg = req.pose_goals

        if not rospy.is_shutdown():
            pose_goals = eepg.ee_poses
            header = eepg.header
            num_poses = len(pose_goals)
            if not num_poses == num_chains:
                print(bcolors.FAIL + 'ERROR: Number of pose goals ({}) ' \
                                        'not equal to the number of kinematic chains ({}).  Exiting relaxed_ik_node'.format(num_poses, num_chains))
                return RelaxedIKServiceResponse()

            pos_goals = []
            quat_goals = []

            for p in pose_goals:
                pos_x = p.position.x
                pos_y = p.position.y
                pos_z = p.position.z

                quat_w = p.orientation.w
                quat_x = p.orientation.x
                quat_y = p.orientation.y
                quat_z = p.orientation.z

                pos_goals.append([pos_x, pos_y, pos_z])
                quat_goals.append([quat_w, quat_x, quat_y, quat_z])

            xopt = relaxedIK.solve(pos_goals, quat_goals)
            ja = JointAngles()
            ja.header = header
            for x in xopt:
                ja.angles.append(Float32(x))

            resp = RelaxedIKServiceResponse()
            resp.joint_angles = ja

            return resp

if __name__ == "__main__":
    RelaxedIKServiceServer()