#!/usr/bin/env python

# Copyright (c) 2021 taurob GmbH. All rights reserved. Confidential.
# Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com

# Script to set the joint positions for the "transport pose".


import argparse
import sys
import time

import rospy
from gazebo_msgs.srv import (
    GetModelProperties,
    GetModelPropertiesRequest,
    SetJointProperties,
    SetJointPropertiesRequest,
    SetModelConfiguration,
    SetModelConfigurationRequest,
)


# The namespace where the used ROS services reside.
GZ_NAMESPACE = "/gazebo/"

# The name of the robot model.
ROBOT_MODEL_NAME = "robot_description"

# The initial joint positions for different robot names.
JOINT_POSITIONS = {
    "inspector": {
        "arm_joint_1": 1.35,
        "arm_joint_2": 2.0,
    },
    "inspector_longarm": {
        "arm_joint_1": 2.01,
        "arm_joint_2": 3.375,
        "arm_joint_3": 0.16,
        "arm_joint_4": 2.95,
    },
    "ogrip": {
        "arm_joint_1": 4.0,
        "arm_joint_2": 3.4,
        "arm_joint_3": 0.0,
        "arm_joint_4": 3.0,
        "arm_joint_5": 1.4,
    },
    "ogrip2": {
        "arm_joint_1": 3.7,
        "arm_joint_2": 2.0,
        "arm_joint_3": 0.3,
        "arm_joint_4": 3.0,
        "arm_joint_5": 1.5,
    },
    "tracker": {
        "arm_joint_1": 3.1,
        "arm_joint_2": 3.1,
        "arm_joint_3": 5.6,
        "arm_joint_4": 3.3,
    },
    "operator": {
        "arm_joint_1": 3.1,
        "arm_joint_2": 3.1,
        "arm_joint_3": 6.2,
        "arm_joint_4": 3.1,
        "arm_joint_5": 0.0,        
    },    
}

# We had strange errors (both with Gazebo 9 and 11) where
# the simulation would forget the damping factor that is
# configured for a joint after we set the joint position.
# For these models, we reset the damping factor after
# setting the positions.
QUIRKS = {
}


def wait_for_joints(joint_names):
    """Wait until the model is loaded and all joints are available."""

    service_name = GZ_NAMESPACE + "get_model_properties"

    rospy.loginfo("Waiting for ROS service '{}'...".format(service_name))
    rospy.wait_for_service(service_name)

    try:
        service = rospy.ServiceProxy(service_name, GetModelProperties)

        while True:
            req = GetModelPropertiesRequest()
            req.model_name = ROBOT_MODEL_NAME

            resp = service(req)

            if resp.success and all(n in resp.joint_names for n in joint_names):
                rospy.loginfo("All joints available.")
                break

            rospy.loginfo("Not all joints available, retrying in one second...")
            # We don't use 'rospy.sleep()' bacause it shall be possible
            # to call this script even when the simulation is paused.
            time.sleep(1.0)
    except rospy.ServiceException as e:
        rospy.logerr("Error getting robot model information: {}".format(e))


def set_joints(joints):
    """Set the joint positions."""

    service_name = GZ_NAMESPACE + "set_model_configuration"

    rospy.loginfo("Waiting for ROS service '{}'...".format(service_name))
    rospy.wait_for_service(service_name)

    try:
        service = rospy.ServiceProxy(service_name, SetModelConfiguration)

        while True:
            req = SetModelConfigurationRequest()
            req.model_name = "robot_description"
            for name, position in joints.items():
                req.joint_names.append(name)
                req.joint_positions.append(position)

            resp = service(req)

            if resp.success:
                rospy.loginfo("Joint positions changed.")
                break

            rospy.loginfo("Setting joint positions failed, retrying in one second...")
            time.sleep(1.0)
    except rospy.ServiceException as e:
        rospy.logerr("Error setting joint positions: {}".format(e))


def apply_quirks(joint_name):
    """Re-set the damping factor of the given joint."""

    service_name = GZ_NAMESPACE + "set_joint_properties"

    rospy.loginfo("Waiting for ROS service '{}'...".format(service_name))
    rospy.wait_for_service(service_name)

    try:
        service = rospy.ServiceProxy(service_name, SetJointProperties)

        req = SetJointPropertiesRequest()
        req.joint_name = joint_name
        req.ode_joint_config.damping = [0.7]
        req.ode_joint_config.hiStop = [0]
        req.ode_joint_config.loStop = [0]
        req.ode_joint_config.erp = [0.2]
        req.ode_joint_config.cfm = [0]
        req.ode_joint_config.stop_erp = [0]
        req.ode_joint_config.stop_cfm = [0]
        req.ode_joint_config.fudge_factor = [0]
        req.ode_joint_config.fmax = [0]
        req.ode_joint_config.vel = [0]

        resp = service(req)
        if resp.success:
            rospy.loginfo("Reset damping factor of joint '{}'.".format(joint_name))
        else:
            rospy.logerr("Unable to reset damping factor of joint '{}'.".format(joint_name))
    except rospy.ServiceException as e:
        rospy.logerr("Error setting joint positions: {}".format(e))


def main():
    rospy.init_node("initial_joint_angles", anonymous=True)

    parser = argparse.ArgumentParser(
        description="Set initial joint angles of the robot's arm."
    )
    parser.add_argument(
        "--wait", type=float, default=4.0, help="Time to sleep before setting joints."
    )
    parser.add_argument(
        "--noquirks", action="store_true", help="Do not work around strange Gazebo bugs."
    )
    parser.add_argument(
        "--robot_model", type=str, help="Name of the robot model."
    )

    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    if args.robot_model not in JOINT_POSITIONS:
        sys.exit("Unknown robot model '{}'.".format(args.robot_model))

    joints = JOINT_POSITIONS[args.robot_model]

    wait_for_joints(joints.keys())

    # Even if the model is loaded and the joints are here, we sleep for a
    # short amount of time before we set the joints, as otherwise other
    # things that are started (for example ros_control controllers) will
    # reset the joint positions to zero.
    time.sleep(args.wait)

    set_joints(joints)

    if (not args.noquirks) and (args.robot_model in QUIRKS):
        apply_quirks(QUIRKS[args.robot_model])


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
