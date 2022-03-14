#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from copy import copy
from math import sin

rate=10
# works with the default example only because I'm not dragging in their libraries to try and parameterize this.

jscb_count=0
rob_pos=[]
rob_vel=[]

def jointstatecallback(msg: JointState):
    global jscb_count, rob_pos, rob_vel
    jscb_count=jscb_count+1
    rob_pos=list(msg.position)
    rob_pos=list(msg.velocity)

# ROS boilerplate
print("Setting up node roscontrol_test, I will wiggle the second last joint ~10deg each way.")
rospy.init_node('roscontrol_test')
# command = rospy.Publisher("/dsr01m1013/dsr_joint_position_controller/command", Float64MultiArray, queue_size=1)
# rospy.Subscriber("/dsr01m1013/joint_states", JointState, jointstatecallback)
# command = rospy.Publisher("/dsr01h2515/dsr_joint_position_controller/command", Float64MultiArray, queue_size=1)
#command = rospy.Publisher("/dsr_control_node/dsr_joint_position_controller/command", Float64MultiArray, queue_size=1)
#rospy.Subscriber("/dsr_control_node/joint_states", JointState, jointstatecallback)

command = rospy.Publisher("/dsr01h2515/dsr_joint_velocity_controller/command", Float64MultiArray, queue_size=1)
rospy.Subscriber("/dsr01h2515/joint_states", JointState, jointstatecallback)

governor=rospy.Rate(rate)

# spin waiting for a valid read from the robot, 3 times should be more than enough.
print("waiting for joint_states")
while jscb_count<3:
    governor.sleep()    # this loop wouldn't break without a nap.
    pass
print("Got some joint states")
print(rob_pos)


#cmd_pos=Float64MultiArray()
cmd_vel=Float64MultiArray()
#cmd_pos.data=rob_pos    # we dont want any joints to move except the second last.
cmd_vel.data=[0,0,0,0,0,0]   # zero velocity
init_pos=copy(rob_pos)  # without the copy the most recent value was sneaking in and making things wacky.
time=0

print("Commanding a small wiggle")
while not rospy.is_shutdown():
    # generate a signal for the robot to track
    #cmd_pos.data[4]=init_pos[4] + (.2*sin(time))
    cmd_vel.data[4]=.2*sin(time)
    #publish command
    #command.publish(cmd_pos)
    command.publish(cmd_vel)

    print(rob_pos)
    #print(cmd_pos.data)
    print(cmd_vel.data)
    print("")

    # 10Hz
    governor.sleep()
    time=time+(1/rate)