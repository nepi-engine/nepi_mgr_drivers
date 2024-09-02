#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import sys
import subprocess
import time

from nepi_edge_sdk_base import nepi_ros

from std_msgs.msg import String, Int32, Bool, Header
from std_srvs.srv import Empty, EmptyResponse
from nepi_ros_interfaces.msg import DriversStatus, DriverUpdateState, DriverUpdateOrder, DriverUpdateOption
from nepi_ros_interfaces.srv import DriverStatusQuery, DriverStatusQueryResponse, DriverListQuery, DriverListQueryResponse
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryResponse

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

# Do this at the end
#from scipy.signal import find_peaks


NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
DRIVERS_FALLBACK_FOLDER = '/opt/nepi/ros/lib/drivers'
DRIVERS_ACTIVE_FALLBACK_FOLDER = '/opt/nepi/ros/lib/drivers/active_drivers'
DRIVERS_INSTALL_FALLBACK_FOLDER = '/mnt/nepi_storage/nepi_src/drivers'

#########################################

#########################################
# Node Class
#########################################

class NepiDriverMgr(object):

   DRIVER_FOLDER_CHECK_INTERVAL = 5
   save_cfg_if = None
   drivers_dict = None
   drivers_folder = None
   drivers_files = []
   drivers_ordered_list = []
   drivers_active_folder = None
   drivers_active_files = []
   drivers_active_list = []
   drivers_install_folder = None
   drivers_install_files = []
   drivers_install_list = []


    
  def getDriverFilesList(self):
    packages_list = []
    folder = self.drivers_folder
    if folder is not None:
      if os.path.exists(folder):
        [file_list, num_files] = nepi_ros.get_file_list(folder,"py")
    for f in file_list:
      packages_list.append(f.split(".")[0])
    return packages_list
    
    def getActiveDriverFilesList(self):
    packages_list = []
    folder = self.drivers_active_folder
    if folder is not None:
      if os.path.exists(folder):
        [file_list, num_files] = nepi_ros.get_file_list(folder,"py")
    for f in file_list:
      packages_list.append(f.split(".")[0])
    return packages_list

    def getDriverPackagesList(self):
    packages_list = []
    folder = self.drivers_install_folder
    if folder is not None:
      if os.path.exists(folder):
        [file_list, num_files] = nepi_ros.get_file_list(folder,"zip")
    for f in file_list:
      packages_list.append(f.split(".")[0])
    return packages_list


 #######################
  ### Node Initialization
  def __init__(self):
    
    node_name = rospy.get_name()
    rospy.loginfo(node_name + ": Starting IF setup")
    # Create a node msg publisher
    self.msg_pub = rospy.Publisher("~messages", String, queue_size=1)
    time.sleep(1)

    # Get driver folder paths
    service_name = nepi_ros.find_service('system_storage_folder_query')
    request = SystemStorageFolderQuery()
    self.drivers_folder = DRIVERS_FALLBACK_FOLDER
    self.drivers_active_folder = DRIVERS_INSTALL_FALLBACK_FOLDER
    self.drivers_install_folder = DRIVERS_INSTALL_FALLBACK_FOLDER
    if service_name != "":
      try:
        request.type = 'drivers'
        response = rospy.ServiceProxy('system_storage_folder_query', request)
        if os.path.exists(response.folder_path):
          self.drivers_folder = response.folder_path
      except:
        self.publishMsg("Failed to get drivers folder from system storage query")
      self.publishMsg("Driver folder set to %s",self.drivers_folder)
      self.drivers_files = self.getDriverFilesList(self.drivers_folder)
      self.publishMsg("Driver folder files %s",str(self.drivers_files))


      try:
        request.type = 'drivers_active'
        response = rospy.ServiceProxy('system_storage_folder_query', SystemStorageFolderQuery)
        if os.path.exists(response.folder_path):
          self.drivers_active_folder = response.folder_path
      except:
        self.publishMsg("Failed to get active drivers folder from system storage query")
      self.publishMsg("Driver folder set to %s",self.drivers_active_folder)
      self.drivers_active_files = self.getDriverActiveFilesList(self.drivers_active_folder)
      self.publishMsg("Driver active folder files %s",str(self.drivers_active_files))

      try:
        request.type = 'nepi_src/drivers'
        response = rospy.ServiceProxy('system_storage_folder_query', SystemStorageFolderQuery)
        if os.path.exists(response.folder_path):
          self.drivers_install_folder = response.folder_path
      except:
        self.publishMsg("Failed to get install drivers folder from system storage query")
      self.publishMsg("Driver folder set to %s",self.drivers_install_folder)
      self.drivers_install_files = self.getDriverPackagesList(self.drivers_install_folder)
      self.publishMsg("Driver install packages folder files %s",str(self.drivers_install_files))






    # Setup message publisher and init param server
    self.publishMsg("Starting Initialization Processes")
    self.initParamServerValues(do_updates = False)
    self.resetParamServer(do_updates = False)
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)

    self.save_cfg_if.saveConfig() # Save config after initialization for next time




    ## Mgr Setup ########################################################
    mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
    self.initParamServerValues(do_updates=False)

    # Setup Service Calls
    rospy.Service('~drivers_list_query',  DriversListQuery, self.provideDriversLists)
    rospy.Service('~driver_status_query', DriverStatusQuery, self.provideDriverStatus)

    # Drivers Management Scubscirbers
    rospy.Subscriber('~update_state', DriverUpdateState, self.updateStateCb)
    rospy.Subscriber('~update_order', DriverUpdateOrder, self.updateOrderCb)
    rospy.Subscriber('~update_option', DriverUpdateOption, self.updateOptionCb)

    rospy.Subscriber('~install_driver', String, self.installDriverCb)
    rospy.Subscriber('~remove_driver', String, self.removeDriverCb)

    # Setup Node Status Publisher
    self.status_pub = rospy.Publisher("~status", DriversStatus, queue_size=1, latch=True)
    # Setup a driver folder timed check
    rospy.Timer(rospy.Duration(self.DRIVER_FOLDER_CHECK_INTERVAL), self.checkDriverFoldersCb)
    time.sleep(1)
    ## Initiation Complete
    self.publish_status()
    self.publishMsg("Initialization Complete")
        
        
    def publishMsg(self,msg):
      msg_str = (self.node_name + ": " + str(msg))
      rospy.loginfo(msg_str)
      if self.msg_pub.getNumSubscribers() > 0:
        self.msg_pub.publish(msg_str)

  
  ###################
  ## Drivers Mgr Services


  def provideDriversLists(self,req_msg):
    drivers_dict = rospy.get_param("~drivers_dict",self.init_drivers_dict)
    res_msg = DriverListQueryResponse()
    res_msg.drivers_path = self.drivers_folder
    res_msg.drivers_ordered_list = nepi_nex.getDriversOrderedList(drivers_dict)
    res_msg.drivers_active_path = self.drivers_active_folder
    res_msg.drivers_active_list = nepi_nex.getDriversActiveOrderedList(drivers_dict)
    res_msg.drivers_install_path = self.drivers_install_folder
    res_msg.drivers_install_list = self.drivers_install_files
    return res_msg


  def provideDriverStatus(self,req_msg):
    drivers_dict = rospy.get_param("~drivers_dict",self.init_drivers_dict)
    driver_name = req_msg.driver_name
    res_msg = DriverStatusQueryResponse()
    if driver_name in drivers_dict.keys():
      driver = drivers_dict[driver_name]
      res_msg.name = driver_name
      res_msg.group = driver['group']
      res_msg.group_id  = driver['group_id']
      res_msg.interfaces  = driver['driver_interfaces']
      res_msg.options  = driver['driver_options']
      res_msg.set_option  = driver['driver_set_option']
      res_msg.discovery = driver['discovery_method']
      res_msg.other_users_list  = driver['users']
      res_msg.active_state  = driver['active']
      res_msg.order  = driver['order']
    return res_msg

  

  ###################
  ## Drivers Mgr Callbacks

  def addAllClassesCb(self,msg):
    ##rospy.loginfo(msg)
    classes = self.current_classifier_classes
    depth = rospy.get_param('~default_target_depth',self.init_default_target_depth)
    selected_dict = dict()
    for Class in classes:
      selected_dict[Class] = {'depth': depth }
    rospy.set_param('~selected_classes_dict', selected_dict)
    self.publish_status()


  def removeAllClassesCb(self,msg):
    ##rospy.loginfo(msg)
    rospy.set_param('~selected_classes_dict', dict())
    self.publish_status()

  def addClassCb(self,msg):
    ##rospy.loginfo(msg)
    class_name = msg.data
    class_depth_m = rospy.get_param('~default_target_depth',  self.init_default_target_depth)
    if class_name in self.current_classifier_classes:
      selected_classes_dict = rospy.get_param('~selected_classes_dict', self.init_selected_classes_dict)
      selected_classes_dict[class_name] = {'depth': class_depth_m}
      rospy.set_param('~selected_classes_dict', selected_classes_dict)
    self.publish_status()


  def removeClassCb(self,msg):
    ##rospy.loginfo(msg)
    class_name = msg.data
    selected_classes_dict = rospy.get_param('~selected_classes_dict', self.init_selected_classes_dict)
    if class_name in selected_classes_dict.keys():
      del selected_classes_dict[class_name]
      rospy.set_param('~selected_classes_dict', selected_classes_dict)
    self.publish_status()

  def selectTargetCb(self,msg):
    ##rospy.loginfo(msg)
    target_name = msg.data
    if target_name == 'All' or target_name in self.current_targets_dict.keys():
      self.selected_target = target_name
    self.publish_status()

  def setVertFovCb(self,msg):
    ##rospy.loginfo(msg)
    fov = msg.data
    if fov > 0:
      rospy.set_param('~image_fov_vert',  fov)
    self.publish_status()


  def setHorzFovCb(self,msg):
    ##rospy.loginfo(msg)
    fov = msg.data
    if fov > 0:
      rospy.set_param('~image_fov_horz',  fov)
    self.publish_status()
    
  def setTargetBoxPercentCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 10 and val <= 200:
      rospy.set_param('~target_box_percent',val)
    self.publish_status()   


  def setPcBoxPercentCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 10 and val <= 200:
      rospy.set_param('~pc_box_percent',val)
    self.publish_status() 
      
  def setDefaultTargetDepthCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 0:
      rospy.set_param('~default_target_depth',val)
    self.publish_status()   

  def setTargetMinPointsCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 0:
      rospy.set_param('~target_min_points',val)
    self.publish_status() 

  def setTargetMinPxRatioCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 0 and val <= 1:
      rospy.set_param('~target_min_px_ratio',val)
    self.publish_status() 

  def setTargetMinDistMCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 0:
      rospy.set_param('~target_min_dist_m',val)
    self.publish_status() 

  def setAgeFilterCb(self,msg):
    #rospy.loginfo(msg)
    val = msg.data
    if val >= 0:
      rospy.set_param('~target_age_filter',val)
    self.publish_status()


   #######################
  ### Mgr Config Functions

  def checkDriverFoldersCb(self,timer):
    need_update = False
    drivers_dict = rospy.get_param("~drivers_dict",self.init_drivers_dict)
    drivers_files = self.getDriverFilesList(self.drivers_folder)
    drivers_active_files = self.getDriverActiveFilesList(self.drivers_active_folder)
    need_update = (self.driver_files != driver_files or self.drivers_active_files != drivers_active_files)
    self.drivers_install_files = self.getDriverPackagesList(self.drivers_install_folder)
    if need_update:
      self.drivers_files = driver_files
      self.drivers_active_files = drivers_active_files
      drivers_dict = rospy.get_param("~drivers_dict",self.init_drivers_dict)
      if drivers_dict is not None:
        drivers_dict = nepi_nex.updateDriversDict(self.drivers_folder,drivers_dict)
      else:
        drivers_dict = nepi_nex.getDriversgetDriverDict(self.drivers_path)
        drivers_dict = nepi_nex.setFactoryDriverOrder(drivers_dict)
        drivers_dict = activateAllDrivers(drivers_dict)

  def resetMgrCb(self,msg):
    self.resetMgr()

  def resetMgr(self):
    # reset drivers dict
    drivers_dict = nepi_nex.getDriversgetDriverDict(self.drivers_path)
    drivers_dict = nepi_nex.setFactoryDriverOrder(drivers_dict)
    drivers_dict = activateAllDrivers(drivers_dict)
    rospy.set_param("~drivers_dict",drivers_dict)

    self.publish_status()

  def saveConfigCb(self, msg):  # Just update Class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    self.initParamServerValues(do_updates = False)

  def updateFromParamServer(self):
    #rospy.logwarn("Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functions
    pass

  def initParamServerValues(self,do_updates = True):
      self.publishMsg("Setting init values to param values")
      # Initialize drivers_dict
      drivers_dict = rospy.get_param("~drivers_dict",None)
      if drivers_dict is not None:
        drivers_dict = nepi_nex.updateDriversDict(self.drivers_folder,drivers_dict)
      else:
        drivers_dict = nepi_nex.getDriversgetDriverDict(self.drivers_path)
        drivers_dict = nepi_nex.setFactoryDriverOrder(drivers_dict)
        drivers_dict = activateAllDrivers(drivers_dict)
      self.init_drivers_dict = drivers_dict
      self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
      rospy.set_param("~drivers_dict",self.init_drivers_dict)
      if do_updates:
          self.updateFromParamServer()
          self.publish_status()

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.publishMsg("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  node_name = "driver_mgr"
  rospy.init_node(name=node_name)
  #Launch the node
  rospy.loginfo("DRV_MGR: Launching node named: " + node_name)
  node = NepiDriverMgr()
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()






