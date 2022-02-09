from math import sin
from time import sleep

import rospy
import dsr_msgs

# Python --  ros services might not be the best way to implement this, even as a dirty test - servoj_rt or speedj_rt services missing

# TURN ON "Real-Time External Control Mode" in setting -- (end of list) Real time external control mode

# connect_rt_control    
#  ip_address =  192.168.137.100
#  port = 12345
drt_connect=rospy.ServiceProxy('realtime/connect_rt_control', dsr_msgs.ConnectRTControl)
retval=drt_connect(ip_address =  "192.168.137.100", port = 12345)
if not retval:
   raise SystemExit('realtime connect failed')

# set_rt_control_output
#  version = "v1.0"
#  period = 0.01    //sampling time, here 100Hz
#  loss = 10     //unknown, currently unused by doosan firmware.
drt_setout=rospy.ServiceProxy('realtime/set_rt_control_output', dsr_msgs.SetRTControlOutput)
retval=drt_setout( version = "v1.0", period = 0.01, loss = 10)
if not retval:
   raise SystemExit('realtime set output failed')


# set up read,  write, stop and disconnect proxies
drt_read=rospy.ServiceProxy('realtime/read_data_rt', dsr_msgs.ReadDataRT)
drt_write=pub = rospy.Publisher('servoj_rt_stream', dsr_msgs.ServoJRTStream, queue_size=1)
readdata=dsr_msgs.RobotStateRT()
writedata=dsr_msgs.ServoJRTStream()
writedata.vel=[0,0,0,0,0,0]
writedata.acc=[0,0,0,0,0,0]
writedata.time=0

thetime=0

# start_rt_control!!
drt_start=rospy.ServiceProxy('realtime/start_rt_control', dsr_msgs.StartRTControl)
retval=drt_start()
if not retval:
   raise SystemExit('realtime start control failed')

readdata=drt_read()
init_pos=readdata.data.actual_joint_position_abs

# -------------main loop ------------------
while not rospy.is_shutdown():
   
   # read_data_rt  //all the data you could ever want, and some you definitly dont
   readdata=drt_read()

   writedata.pos=init_pos
   writedata.pos[4]=init_pos[4]+0.1*sin(thetime)
   # use servoj_rt instead for position control, which isnt exposed as a service, but as a topic subscriber
   drt_write.publish(writedata)

   sleep(0.01) #ros later
   thetime=thetime+0.01

# ----------------CLEANUP-------------

# stop_rt_control
drt_stop=rospy.ServiceProxy('realtime/stop_rt_control', dsr_msgs.StopRTControl)
retval = drt_stop()
# no if, because when it fails... well there's not much i can do to stop it now is there?
print("Stop returns: " + str(retval))

# disconnect_rt_control_cb
drt_drop=rospy.ServiceProxy('realtime/disconnect_rt_control', dsr_msgs.DisconnectRTControl)
retval = drt_drop()
print("Disconnect returns: " + str(retval))
