#!/usr/bin/env python
import rospyimport math
#from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from prius_msgs.msg import Control
from nav_msgs.msg import Path, Odometry
index=0
vt=5.
vf=0.
vfx=0
p_error=0.
err_sum=0.
yaw=0.
r_x=0.
r_y=0.
kd=0.3
delta=0
path=list()
ctrl=Control()
num=0

def main():
    rospy.init_node("pidpp",anonymous=True)
    #while not rospy.is_shutdown():
    pub=rospy.Publisher("prius",Control, queue_size=5)
    sub1=rospy.Subscriber("astroid_path", Path, path_callback)
    sub2=rospy.Subscriber("base_pose_ground_truth", Odometry, odo_callback,pub)
#    global ctrl
#   rate = rospy.Rate(20)
    
#    while not rospy.is_shutdown:
#		print("Up and Working!")
#		purepursuit() 
        #ismein pure_pursuit aur pid dono hai
#		pub.publish(ctrl)
#		print "published"
#		rate.sleep()
    rospy.spin()
    
def path_callback(path_msg):
    global path
    print "path_aaya"
    path=path_msg.poses
    num=1

def odo_callback(odo_msg,pub):
    global vf, yaw, r_x, r_y, ctrl, num
    print "odo_aaya"
    vf=math.hypot(odo_msg.twist.twist.linear.x, odo_msg.twist.twist.linear.y)
    ori=odo_msg.pose.pose.orientation
    pitch,roll,yaw=euler_from_quaternion([ori.x,ori.y,ori.z,ori.w])
    r=odo_msg.pose.pose.position
    r_x=r.x
    r_y=r.y
    ctrl.header=odo_msg.header
	print(num)
    if(num==1):
        purepursuit()
        pub.publish(ctrl)
        print("published")
	
    
def pid():
    global p_error,vt,vf,err_sum,ctrl
    error=vt-vf
    diff=error-p_error
    p_error=error
    err_sum=err_sum+error
    #kp=3 ki=0.05 kd=0.05
    thr=(3.0*error+.05*diff+.05*err_sum)/15.0
    if (thr>1.): return (1,0)
    elif(thr>=0.):return (thr,0)
    elif (thr>=-1.):return(0,-thr)
    else : return (0,1)
    
def purepursuit():
    global path,index,r_x,r_y,kd,vf,yaw,ctrl
    i=index
    np=math.hypot(path[i].pose.position.x-r_x, path[i].pose.position.y-r_y)
    dis=math.hypot(path[i+1].pose.position.x-r_x, path[i+1].pose.position.y-r_y)
    
    while (dis<np and i<len(path)):
        i=i+1
        np=dis
        dis=math.hypot(path[i+1].pose.position.x-r_x, path[i+1].pose.position.y-r_y)
    index=i
    while(np<math.fabs(kd*vf)):
        i=i+1
        np=math.hypot(path[i].pose.position.x-r_x, path[i].pose.position.y-r_y)
        
    g_x=path[i].pose.position.x-r_x
    g_y=path[i].pose.position.y-r_y
    g_xtrf=g_x*math.cos(yaw)+g_y*math.sin(yaw)
    g_ytrf=-g_x*math.sin(yaw)+g_y*math.cos(yaw)
    
    alpha=math.atan(g_ytrf/g_xtrf)
    #if (alpha!=0) : sign=alpha/math.fabs(alpha)
    #else: sign=1
    
    delta=math.atan(2*1.45*math.sin(alpha)/0.1+kd*vf)
    st=delta/.698
    if (st>1.): st=1.
    elif(st<-1.):st=-1.
    
    ctrl.steer=st
    (thr,brk)=pid()
    ctrl.throttle=thr
    ctrl.brake=brk
    ctrl.shift_gears=2
    
if __name__=="__main__":
    try:
		main()
	except rospy.ROSInterruptException:
		pass
    
    
    
    
    
