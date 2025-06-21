#!/usr/bin/env python3

import numpy as np
import os
import sys
import open3d as o3d
import roslib
import rospy
import tf
import tf2_ros
from scipy.spatial.transform import Rotation as rot
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from rovi_utils import tflib

Config={
  "model":"model.ply",
  "frame_id":"base",
  "tr":[20,-935,411, -0.0129,-0.999,-0.005,-0.002]
}

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

  scn=np.vstack((scn,pn))
  return scn

def cb_redraw(msg):
  pub_wp.publish(np2F(Scene))


########################################################
rospy.init_node("venv",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/venv"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
pub_wp=rospy.Publisher("/rovi/wp_floats",numpy_msg(Floats),queue_size=1)
pub_err=rospy.Publisher("/rsim/error",Int32,queue_size=1)
pub_report=rospy.Publisher("/report",String,queue_size=1)
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
###Global
mError=Int32()
Scene=[]  #will be point cloud ndarray as [[x,y,z]...]
Stack=[]  #will be as [{"tf":[...],"xo":[...],"wp":[...]}...]
Pcd=[]
Pcd.append(o3d.io.read_point_cloud(thispath+'/'+Config['model']))
Pcd.append(o3d.io.read_point_cloud(thispath+'/'+Config['environ']))
#Pcd.append(o3d.io.read_point_cloud(thispath+'/'+Config['diff']))
if len(Pcd)>1:
  Pcd[-1].transform(tflib.toRTfromVec(Config["env_tr"]))
#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
