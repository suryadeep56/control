#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
#from prius_msgs.msg import Control
from nav_msgs.msg import Path, Odometry
index=0
#vt=5.
v=0.
vfx=0
p_error=0.
err_sum=0.
yaw=0.
r_x=0.
r_y=0.
kd=0.1
delta=0
path=list()
pub=rospy.Publisher("action",Twist, queue_size=1)
num=0

def main():
    rospy.init_node("pidpp",anonymous=True)
    #while not rospy.is_shutdown():
    #pub=rospy.Publisher("action",Twist, queue_size=5)
    sub1=rospy.Subscriber("astroid_path", Path, path_callback)
    sub2=rospy.Subscriber("base_pose_ground_truth", Odometry, odo_callback)
    global ctrl
    #rate = rospy.Rate(20)
    
    #while not rospy.is_shutdown:
		#print("Up and Working!")
		#purepursuit() 
        #ismein pure_pursuit aur pid dono hai
		#pub.publish(ctrl)
		#print "published"
		#rate.sleep()
    rospy.spin()
    
def path_callback(path_msg):
    global path, num
    #print "path_aaya"
    path=path_msg.poses
    num=1

def odo_callback(odo_msg):
    global v
    global yaw
    global r_x, r_y, pub,num
    #print "odo_aaya"
    v=math.hypot(odo_msg.twist.twist.linear.x, odo_msg.twist.twist.linear.y)
    #ori=odo_msg.pose.pose.orientation
    #yaw=math.atan2(2.0*(ori.y*ori.z+ori.w*ori.x),ori.w*ori.w-ori.x*ori.x-ori.y*ori.y+ori.z*ori.z)
    #print(yaw)
    yaw=odo_msg.twist.twist.angular.z
    r=odo_msg.pose.pose.position
    r_x=r.x
    r_y=r.y
    twst=Twist()
    if(num==1):
        purepursuit(twst)
        pub.publish(twst)
        print("published",twst)

def purepursuit(twst):
    global path,index,r_x,r_y,kd,v,yaw,pub
    print("------------------------------------------PP SHURU HUA--------------------------------------------")
    i=index
    vel_flag=0
    if i<(len(path)-2) :
        np=math.hypot(path[i].pose.position.x-r_x, path[i].pose.position.y-r_y)
        dis=math.hypot(path[i+1].pose.position.x-r_x, path[i+1].pose.position.y-r_y)
    #print(np,dis,yaw)
        while (dis<np and i<len(path)-1):
            i=i+1
            np=dis
            dis=math.hypot(path[i+1].pose.position.x-r_x, path[i+1].pose.position.y-r_y)
        index=i+1
        while(np<math.fabs(kd*v)):
            i=i+1
            np=math.hypot(path[i].pose.position.x-r_x, path[i].pose.position.y-r_y)

        #80;9print("np:",np,"th:",kd*v)-
            
        g_x=path[i].pose.position.x-r_x
        g_y=path[i].pose.position.y-r_y
        g_xtrf=g_x*math.cos(yaw)+g_y*math.sin(yaw)
        g_ytrf=-g_x*math.sin(yaw)+g_y*math.cos(yaw) 
        alpha=math.atan(g_ytrf/g_xtrf)
    
    
        #vel_flag=1
    
        delta=math.atan(2*2.5*math.sin(alpha)/(0.0001+kd*v))
        MAX_STEER=math.pi/4
        if delta >= MAX_STEER:
            delta = MAX_STEER
        elif delta <= -MAX_STEER:
            delta = -MAX_STEER
        omega = math.tan(delta)*v/2.5 #2.5m is wheelbase
        
        twst.linear.x=5*math.cos(yaw)
        twst.linear.y=5*math.sin(yaw)
        twst.angular.z=omega

    else:
        twst.linear.x=0
        twst.linear.y=0
        twst.angular.z=0  
    
    print("----------------------------------------------PP KHATAM HUA--------------------------------------------------")
if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
    
    
    
    
