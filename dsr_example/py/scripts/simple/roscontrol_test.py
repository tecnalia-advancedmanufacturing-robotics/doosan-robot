#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin
import moveit_commander
import signal, sys
import numpy as np
import dsr_msgs.msg

rate=10
# works with the default example only because I'm not dragging in their libraries to try and parameterize this.

jscb_count=0
rob_pos=[]
rob_vel=[]

moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander("manipulator")

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# ROS boilerplate
print("Setting up node roscontrol_test, I will wiggle the second last joint ~10deg each way.")
rospy.init_node('roscontrol_test')
# command = rospy.Publisher("/dsr01m1013/dsr_joint_position_controller/command", Float64MultiArray, queue_size=1)
# rospy.Subscriber("/dsr01m1013/joint_states", JointState, jointstatecallback)
# command = rospy.Publisher("/dsr01h2515/dsr_joint_position_controller/command", Float64MultiArray, queue_size=1)
#command = rospy.Publisher("/dsr_control_node/dsr_joint_position_controller/command", Float64MultiArray, queue_size=1)
#rospy.Subscriber("/dsr_control_node/joint_states", JointState, jointstatecallback)

state = None
def state_cb(state_msg):
    global state
    state=state_msg

rospy.Subscriber("/m1509/state", dsr_msgs.msg.RobotState, state_cb)

command = rospy.Publisher("/m1509/dsr_joint_velocity_controller/command", Float64MultiArray, queue_size=1)

governor=rospy.Rate(rate)


#cmd_pos=Float64MultiArray()
cmd_vel=Float64MultiArray()
#cmd_pos.data=rob_pos    # we dont want any joints to move except the second last.
cmd_vel.data=[0,0,0,0,0,0]   # zero velocity
time=0

def sigint_handler(s,f):
    command.publish(data=[0,0,0,0,0,0])
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

print("Commanding a small wiggle")
while not rospy.is_shutdown():
    # generate a signal for the robot to track
    #cmd_pos.data[4]=init_pos[4] + (.2*sin(time))
    # cmd_vel.data[4]=.2*sin(time)


    amp=0.01
    vel=[0,0,amp*sin(rospy.Time.now().to_sec()),0,0,0]

    if state is not None:
        print(state.fWorldETT)
        vel[2]=state.fWorldETT[2]/500
        # print("vel3")

    joints=move_group.get_current_joint_values()
    jacobian=move_group.get_jacobian_matrix(joints)
    jinv = np.linalg.pinv(jacobian)
    velj=np.matmul(jinv,np.array(vel))
    # print("velj="+(velj).__repr__())
    cmd_vel.data=list(velj)
    #publish command
    #command.publish(cmd_pos)
    command.publish(data=list(velj))

    #print(cmd_pos.data)
    print(cmd_vel.data)
    print("")

    # 10Hz
    governor.sleep()
    time=time+(1./rate)

cmd_vel.data=[0,0,0,0,0,0]   # zero velocity
command.publish(cmd_vel)