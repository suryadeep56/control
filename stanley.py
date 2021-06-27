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
kd=.1
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
    rospy.spin()
    
def path_callback(path_msg):
    global path, num
    #print "path_aaya"
    path=path_msg.poses
    num=1

def odo_callback(odo_msg):
    global v
    global yaw
    global r_x, r_y, pub
    #print "odo_aaya"
    v=math.hypot(odo_msg.twist.twist.linear.x, odo_msg.twist.twist.linear.y)
    #ori=odo_msg.pose.pose.orientation
    #yaw=math.atan2(2.0*(ori.y*ori.z+ori.w*ori.x),ori.w*ori.w-ori.x*ori.x-ori.y*ori.y+ori.z*ori.z)
    #print(yaw)
    yaw=odo_msg.twist.twist.angular.z
    r=odo_msg.pose.pose.position
    r_x=r.x+2.5*math.cos(yaw)
    r_y=r.y+2.5*math.sin(yaw)
    twst=Twist()
    if(num==1):
        try:
            stanley(twst)
            pub.publish(twst)
            print("published",twst)
        except:
            twst.linear.x=0.0
            twst.linear.y=0.0
            twst.angular.z=0.0
            pub.publish(twst)
            print("published",twst)

def stanley(twst):
    global path,index,r_x,r_y,kd,v,yaw,pub
    print("-----------------------------------------------------------------------------------")
    i=index
    vel_flag=0
    if i<(len(path)-3) :
        np=math.hypot(path[i].pose.position.x-r_x, path[i].pose.position.y-r_y)
        dis=math.hypot(path[i+1].pose.position.x-r_x, path[i+1].pose.position.y-r_y)
        print(np,dis,yaw)
        while (dis<np and i<len(path)-1):
            i=i+1
            np=dis
            dis=math.hypot(path[i+1].pose.position.x-r_x, path[i+1].pose.position.y-r_y)
        index=i-1
        print(np,dis,yaw)
        #print("np:",np,"th:",kd*v)-
    
        #g_x=path[i].pose.position.x-r_x
        #g_y=path[i].pose.position.y-r_y
        #g_xtrf=g_x*math.cos(yaw)+g_y*math.sin(yaw)
        #g_ytrf=-g_x*math.sin(yaw)+g_y*math.cos(yaw) 
        #alpha=math.atan(g_ytrf/g_xtrf)
        #vel_flag=1

        qx=path[i].pose.orientation.x
        qy=path[i].pose.orientation.y
        qz=path[i].pose.orientation.z
        qw=path[i].pose.orientation.w
        
        eul=euler_from_quaternion([qx,qy,qz,qw])
        path_yaw=eul[2]
        print(path_yaw)
        sign=1
        if path_yaw<0: sign=-1
        delta=(-yaw+path_yaw)+math.atan(kd*np*sign/(0.0001+v))
        MAX_STEER=math.pi/4
        if delta >= MAX_STEER:
            delta = MAX_STEER
        elif delta <= -MAX_STEER:
            delta = -MAX_STEER
        omega = math.tan(delta)*v/2.5 #2.5m is wheelbase
        
        twst.linear.x=1.5*math.cos(yaw)
        twst.linear.y=1.5*math.sin(yaw)
        twst.angular.z=omega

    else:
        twst.linear.x=0.0
        twst.linear.y=0.0
        twst.angular.z=0.0  
    
    print("----------------------------------------------------------------------------------------------")
if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
    
    
    
    
