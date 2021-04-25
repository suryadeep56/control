#!/usr/bin/env python
import rospy
import math
import control
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
#from prius_msgs.msg import Control
from nav_msgs.msg import Path, Odometry
index=0
vt=4
v=0.
vfx=0
dt=0.05
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
p_theta=0
p_crstr=0
dt=0.05
prev_delta=0
iterate=0
prev_path_yaw=0

def main():
    rospy.init_node("pidpp",anonymous=True)
    #while not rospy.is_shutdown():
    #pub=rospy.Publisher("action",Twist, queue_size=5)
    sub1=rospy.Subscriber("astroid_path", Path, path_callback)
    sub2=rospy.Subscriber("base_pose_no_error", Odometry, odo_callback)
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
    global r_x, r_y, pub
    #print "odo_aaya"
    v=math.hypot(odo_msg.twist.twist.linear.x, odo_msg.twist.twist.linear.y)
    #ori=odo_msg.pose.pose.orientation
    #yaw=math.atan2(2.0*(ori.y*ori.z+ori.w*ori.x),ori.w*ori.w-ori.x*ori.x-ori.y*ori.y+ori.z*ori.z)
    #print(yaw)
    yaw=odo_msg.twist.twist.angular.z
    r=odo_msg.pose.pose.position
    r_x=r.x+1.25*math.cos(yaw)
    r_y=r.y+1.25*math.sin(yaw)
    twst=Twist()
    if(num==1):
        lqr(twst)
        pub.publish(twst)
        print("published",twst)

def lqr(twst):
    global path,index,r_x,r_y,kd,v,yaw,pub,vt,p_theta,p_crstr,dt,prev_delta,iterate, prev_path_yaw
    print("------------------------------------------ SHURU HUA--------------------------------------------")
    i=index
    p_x=path[i].pose.position.x
    vel_flag=0
    if i<(len(path)-3) :
        crstr=math.hypot(path[i].pose.position.x-r_x, path[i].pose.position.y-r_y)
        dis=math.hypot(path[i+1].pose.position.x-r_x, path[i+1].pose.position.y-r_y)
        print(crstr,dis,yaw)
        while (dis<crstr and i<len(path)-1):
            i=i+1
            crstr=dis
            dis=math.hypot(path[i+1].pose.position.x-r_x, path[i+1].pose.position.y-r_y)
        index=i-1
        print(crstr,dis,yaw)
        #80;9print("crstr:",crstr,"th:",kd*v)-
        x=path[i].pose.position.x
        qx=path[i].pose.orientation.x
        qy=path[i].pose.orientation.y
        qz=path[i].pose.orientation.z
        qw=path[i].pose.orientation.w
        
        eul=euler_from_quaternion([qx,qy,qz,qw])
        path_yaw=eul[2]

        #dx=path[i+1].pose.position.x-path[i].pose.position.x
        
        #qx=path[i+1].pose.orientation.x
        #qy=path[i+1].pose.orientation.y
        #qz=path[i+1].pose.orientation.z
        #qw=path[i+1].pose.orientation.w

        #eul=euler_from_quaternion([qx,qy,qz,qw])
        #nex_path_yaw=eul[2]

        if (p_x-x)!=0: curv=((math.tan(path_yaw)-math.tan(prev_path_yaw))/(p_x-x))/((1+math.tan(path_yaw)**2)**1.5)
        else: curv=0
        prev_path_yaw=path_yaw
        theta=path_yaw-yaw

        A = np.zeros((4, 4))
        A[0,0]=1
        A[0,1]=dt
        #A[1,1]=
        A[1,2]=v#*(1-(theta**2)/3)
        A[2,2]=1
        A[2,3]=dt
        #A[3,3]=

        B = np.zeros((4, 1))
        #B[1,0]=136.4*dt        
        B[3,0] = v/2.5

        x = np.zeros((4, 1))
        x[0,0] =crstr
        if iterate==0: x[1,0] = 0
        else: x[1,0] =(crstr-p_crstr)/dt
        x[2,0] =theta
        if iterate==0: x[3,0] = 0
        else: x[3,0] =(theta-p_theta)/dt

        Q=np.eye(4)
        Q[0,0]=15
        Q[1,1]=.5
        Q[2,2]=1
        Q[3,3]=0.5
        R=np.eye(1)
        R[0,0]=1

        p_theta=theta
        p_crstr=crstr

        X,L,G=control.dare(A,B,Q,R)
        
        print(G)
        
        if iterate==1: delta=-(np.matmul(G,x))
        else: delta=-(np.matmul(G,x))+2.5*curv#+prev_delta
        delta=(delta + math.pi) % (2 * math.pi) - math.pi
        prev_delta=delta
        print(delta)

        MAX_STEER=math.pi/4
        if delta >= MAX_STEER:
            delta = MAX_STEER
        elif delta <= -MAX_STEER:
            delta = -MAX_STEER
        omega = math.tan(delta)*v/2.5 #2.5m is wheelbase
        
        twst.linear.x=vt*math.cos(yaw)
        twst.linear.y=vt*math.sin(yaw)
        twst.angular.z=omega
        iterate=iterate+1

    else:
        twst.linear.x=0.0
        twst.linear.y=0.0
        twst.angular.z=0.0  
    
    print("---------------------------------------------- KHATAM HUA--------------------------------------------------")
if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
    
    
    
    
