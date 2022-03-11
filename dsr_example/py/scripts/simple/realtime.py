#!/usr/bin/env python3
from math import sin
from time import sleep

import rospy
from sensor_msgs.msg import JointState
from dsr_msgs.srv import *
from dsr_msgs.msg import SpeedJRTStream, ServoJRTStream, RobotStateRT

rospy.init_node("realtime_test")

# connect_rt_control    
#  ip_address =  192.168.137.100
#  port = 12347

## bryans feature_rt_roscontrol dsr driver does an rt connect already ##

# drt_connect=rospy.ServiceProxy('/dsr01h2515/realtime/connect_rt_control', ConnectRTControl)
# retval=drt_connect(ip_address =  "192.168.137.100", port = 12347)  driver alrady connects
# if not retval:
#    raise SystemExit('realtime connect failed')

# set_rt_control_output
#  version = "v1.0"
#  period = 0.01    //sampling time, here 100Hz
#  loss = 10     //unknown, currently unused by doosan firmware.
drt_setout=rospy.ServiceProxy('/dsr01h2515/realtime/set_rt_control_output',SetRTControlOutput)
retval=drt_setout( version = "v1.0", period = 0.01, loss = 10)
if not retval:
   raise SystemExit('realtime set output failed')

roboang=[0,0,0,0,0,0]
robovel=[0,0,0,0,0,0]
read_cb_count=0
# set up read,  write, stop and disconnect proxies
def read_cb(data: JointState):
   global roboang, robovel, read_cb_count
   roboang=list(data.position)
   robovel=list(data.velocity)
   #print(roboang)
   read_cb_count=read_cb_count+1

rospy.Subscriber("/dsr01h2515/joint_states", JointState, read_cb) # driver doesnt expose rt data as topic yet
# drt_write=pub = rospy.Publisher('/dsr01h2515/servoj_rt_stream', ServoJRTStream, queue_size=1)
drt_write=pub = rospy.Publisher('/dsr01h2515/speedj_rt_stream', SpeedJRTStream, queue_size=1)
readdata=RobotStateRT()
#writedata=ServoJRTStream()
writedata=SpeedJRTStream()
writedata.vel=[0,0,0,0,0,0]
writedata.acc=[0,0,0,0,0,0]
writedata.time=0

# start_rt_control!!
drt_start=rospy.ServiceProxy('/dsr01h2515/realtime/start_rt_control', StartRTControl)
retval=drt_start()
if not retval:
   raise SystemExit('realtime start control failed')

#set limits
drt_velj_limits=rospy.ServiceProxy('/dsr01h2515/realtime/set_velj_rt', SetVelJRT)
drt_accj_limits=rospy.ServiceProxy('/dsr01h2515/realtime/set_accj_rt', SetAccJRT)
drt_velx_limits=rospy.ServiceProxy('/dsr01h2515/realtime/set_velx_rt', SetVelXRT)
drt_accx_limits=rospy.ServiceProxy('/dsr01h2515/realtime/set_accx_rt', SetAccXRT)

drt_velj_limits([100, 100, 100, 100, 100, 100])
drt_accj_limits([100, 100, 100, 100, 100, 100])
drt_accx_limits(100,10)
drt_velx_limits(200,10)


while read_cb_count<5:
   sleep(0.01)

init_pos=roboang
timestep=0.01
thetime=0
# -------------main loop ------------------
while not rospy.is_shutdown():
   
   # read_data_rt  //all the data you could ever want, and some you definitly dont
   #readdata=drt_read()

   
   writedata.acc=[-10000,-10000,-10000,-10000,-10000,-10000]
   writedata.vel=[0,0,0,0,0,0]
   writedata.time=0.0001
   # writedata.pos[4]=init_pos[4]+0.1*sin(thetime)
   writedata.vel[4]=57*0.1*sin(thetime)
   
   # publish command
   drt_write.publish(writedata)

   sleep(timestep) #ros later
   # print(f"des: {writedata.pos[4]:5.3f}")
   print(f"des: {(writedata.vel[4]/57):5.3f}")
   # print(f"act: {roboang[4]:5.3f}\n")
   print(f"act: {robovel[4]:5.3f}\n")
   thetime=thetime+timestep

# ----------------CLEANUP-------------

# stop motion
writedata.vel=[0,0,0,0,0,0]
drt_write.publish(writedata)

# stop_rt_control
drt_stop=rospy.ServiceProxy('/dsr01h2515/realtime/stop_rt_control', StopRTControl)
retval = drt_stop()
# no if, because when it fails... well there's not much i can do to stop it now is there?
print("Stop returns: " + str(retval))

# disconnect_rt_control_cb
drt_drop=rospy.ServiceProxy('/dsr01h2515/realtime/disconnect_rt_control', DisconnectRTControl)
retval = drt_drop()
print("Disconnect returns: " + str(retval))
