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
import rospy
import sys
import subprocess
import time

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_nex

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_ros_interfaces.msg import DriversStatus, DriverUpdateState, DriverUpdateOrder, DriverUpdateOption
from nepi_ros_interfaces.srv import DriverStatusQuery, DriverStatusQueryResponse
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

  DRIVER_UPDATE_CHECK_INTERVAL = 5
  save_cfg_if = None
  drivers_folder = ''
  drivers_files = []
  drivers_ordered_list = []
  drivers_active_folder = ''
  drivers_active_files = []
  drivers_active_list = []
  drivers_install_folder = ''
  drivers_install_files = []
  drivers_install_list = []

  status_msg = DriversStatus()
  last_status_msg = DriversStatus()
    



  #######################
  ### Node Initialization
  def __init__(self):
    
    self.node_name = rospy.get_name()
    rospy.loginfo(self.node_name + ": Starting IF setup")
    # Create a node msg publisher
    self.msg_pub = rospy.Publisher("~messages", String, queue_size=1)
    time.sleep(1)

    # Get driver folder paths
    self.drivers_folder = DRIVERS_FALLBACK_FOLDER
    self.drivers_active_folder = DRIVERS_INSTALL_FALLBACK_FOLDER
    self.drivers_install_folder = DRIVERS_INSTALL_FALLBACK_FOLDER
    service_name = nepi_ros.find_service('system_storage_folder_query')
    request = SystemStorageFolderQuery()
    # Get Drivers Folder
    try:
      request.type = 'drivers'
      response = rospy.ServiceProxy('system_storage_folder_query', request)
      if os.path.exists(response.folder_path):
        self.drivers_folder = response.folder_path
    except:
      self.publishMsg("Failed to get drivers folder from system storage query")
    self.publishMsg("Driver folder set to " + self.drivers_folder)
    self.drivers_files = nepi_nex.getDriverFilesList(self.drivers_folder)
    self.publishMsg("Driver folder files " + str(self.drivers_files))
    # Get Active Drivers Folder
    try:
      request.type = 'drivers_active'
      response = rospy.ServiceProxy('system_storage_folder_query', SystemStorageFolderQuery)
      if os.path.exists(response.folder_path):
        self.drivers_active_folder = response.folder_path
    except:
      self.publishMsg("Failed to get active drivers folder from system storage query")
    self.publishMsg("Driver folder set to " + self.drivers_active_folder)
    self.drivers_active_files = nepi_nex.getDriverActiveFilesList(self.drivers_active_folder)
    self.publishMsg("Driver active folder files " + str(self.drivers_active_files))
    # Get Install Drivers Folder
    try:
      request.type = 'nepi_src/drivers'
      response = rospy.ServiceProxy('system_storage_folder_query', SystemStorageFolderQuery)
      if os.path.exists(response.folder_path):
        self.drivers_install_folder = response.folder_path
    except:
      self.publishMsg("Failed to get install drivers folder from system storage query")
    self.publishMsg("Driver folder set to " + self.drivers_install_folder)
    self.drivers_install_files = nepi_nex.getDriverPackagesList(self.drivers_install_folder)
    self.publishMsg("Driver install packages folder files " + str(self.drivers_install_files))

    # Setup Node Status Publisher
    self.status_pub = rospy.Publisher("~status", DriversStatus, queue_size=1, latch=True)

    # Setup message publisher and init param server
    self.publishMsg("Starting Initialization Processes")
    self.initParamServerValues(do_updates = False)
    self.resetParamServer(do_updates = False)

    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)
    self.save_cfg_if.saveConfig() # Save config after initialization for next time

 
    ## Mgr ROS Setup ########################################################
    mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
    self.initParamServerValues(do_updates=False)
    self.printND()
    # Setup Service Calls
    rospy.Service('~driver_status_query', DriverStatusQuery, self.provideDriverStatus)

    # Drivers Management Scubscirbers
    rospy.Subscriber('~update_state', DriverUpdateState, self.updateStateCb)
    rospy.Subscriber('~update_order', DriverUpdateOrder, self.updateOrderCb)
    rospy.Subscriber('~update_option', DriverUpdateOption, self.updateOptionCb)

    rospy.Subscriber('~install_driver_pkg', String, self.installDriverPkgCb)
    rospy.Subscriber('~remove_driver', String, self.removeDriverCb)

    # Setup a driver folder timed check
    rospy.Timer(rospy.Duration(self.DRIVER_UPDATE_CHECK_INTERVAL), self.checkDriversDataCb)
    time.sleep(1)
    ## Initiation Complete
    self.publish_status()
    self.publishMsg("Initialization Complete")
        
        
  def publishMsg(self,msg = '',level = 'Info', line_num = None):
    line_str = ''
    if line_num is not None:
      line_str = 'line: ' + str(line_num) + ': ' 
    node_str = self.node_name.split('/')[-1]
    msg_str = (node_str + ": " + line_str +str(msg))
    if level == 'Debug':
      rospy.logdebug(msg_str)
    elif level == 'Warn':
      rospy.logwarn(msg_str)
    elif level == 'Error':
      rospy.logerr(msg_str)
    elif level == 'Fatal':
      rospy.logfatal(msg_str)
    else:
      rospy.loginfo(msg_str)
    if self.msg_pub.get_num_connections() > 0 and not rospy.is_shutdown():
      self.msg_pub.publish(msg_str)


  # line_num = sys._getframe().f_lineno ; self.printND(line_num)
  def printND(self,line_num = None):
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    self.publishMsg('','Warn')
    self.publishMsg('*******************','Warn')
    if line_num is not None:
      self.publishMsg(str(line_num),'Warn')
    for nex_name in nex_database.keys():
      nex_dict = nex_database[nex_name]
      self.publishMsg(str(nex_dict),'Warn')

  def getStatusMsg(self):
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    status_msg = DriversStatus()
    status_msg.drivers_path = self.drivers_folder
    status_msg.drivers_ordered_list = nepi_nex.getDriversOrderedList(nex_database)
    status_msg.drivers_active_path = self.drivers_active_folder
    status_msg.drivers_active_list = nepi_nex.getDriversActiveOrderedList(nex_database)
    status_msg.drivers_install_path = self.drivers_install_folder
    status_msg.drivers_install_list = self.drivers_install_files
    return status_msg

  
  def publish_status(self):
    self.last_status_msg = self.status_msg
    self.status_msg = self.getStatusMsg()
    if not rospy.is_shutdown():
      self.status_pub.publish(self.status_msg)


  def checkDriversDataCb(self,timer):
    need_update = False
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    drivers_files = nepi_nex.getDriverFilesList(self.drivers_folder)
    drivers_active_files = nepi_nex.getDriverActiveFilesList(self.drivers_active_folder)
    need_update = (self.drivers_files != drivers_files or self.drivers_active_files != drivers_active_files)
    self.drivers_install_files = nepi_nex.getDriverPackagesList(self.drivers_install_folder)
    if need_update:
      self.drivers_files = drivers_files
      self.drivers_active_files = drivers_active_files
      nex_database = rospy.get_param("~nex_database",self.init_nex_database)
      if nex_database is not None:
        nex_database = nepi_nex.updateDriversDict(self.drivers_folder,nex_database)
      else:
        nex_database = nepi_nex.getDriversgetDriverDict(self.drivers_folder)
        nex_database = nepi_nex.setFactoryDriverOrder(nex_database)
        nex_database = activateAllDrivers(nex_database)
    rospy.set_param("~nex_database",nex_database)
    print(self.drivers_files)
    print(self.drivers_active_files)
    print(self.drivers_install_files)
    self.publish_status()

    
  

  ###################
  ## Drivers Mgr Callbacks


  def updateStateCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.driver_name
    new_active_state = msg.active_state
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      driver = nex_database[driver_name]
      active_state = driver['active']
    if new_active_state != active_state:
      if new_active_state == True:
        nex_database = nepi_nex.activateDriver(driver_name,nex_database)
      else:
        nex_database = nepi_nex.disableDriver(driver_name,nex_database)
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()


  def updateOrderCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.driver_name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      nex_database = moveFunction(driver_name,nex_database)
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()


  def getOrderUpdateFunction(self,move_cmd):
    if move_cmd == 'top':
      updateFunction = nepi_nex.moveDriverTop
    elif move_cmd == 'bottom':
      updateFunction = nepi_nex.moveDriverBottom
    elif move_cmd == 'up':
      updateFunction = nepi_nex.moveDriverUp
    elif move_cmd == 'down':
      updateFunction = nepi_nex.moveDriverDown
    else:
      updateFunction = self.moveDriverNone
    return updateFunction

  def moveDriverNone(self,driver_name,nex_database):
    return nex_database
    

  def updateOptionCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.driver_name
    option = msg.option_str
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      driver = nex_database[driver_name]
      options = driver['driver_options']
      if option in options:
        driver['driver_set_option'] = option
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()

  def installDriverPkgCb(self,msg):
    self.publishMsg(msg)
    pkg_name = msg.data
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if pkg_name in self.drivers_install_files:
      nex_database = nepi_nex.installDriverPkg(pkg_name,nex_database,self.drivers_install_folder,self.drivers_folder)
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()

  def removeDriverCb(self,msg):
    self.publishMsg(msg)
    diver_name = msg.data
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database:
      nex_database = nepi_nex.removeDriver(driver_name,nex_database)
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()


  ###################
  ## Drivers Mgr Services





  def provideDriverStatus(self,req_msg):
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    driver_name = req_msg.driver_name
    res_msg = DriverStatusQueryResponse()
    if driver_name in nex_database.keys():
      driver = nex_database[driver_name]
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

   #######################
  ### Mgr Config Functions

 

  def resetMgrCb(self,msg):
    self.resetMgr()

  def resetMgr(self):
    # reset drivers dict
    nex_database = nepi_nex.getDriversgetDriverDict(self.drivers_folder)
    nex_database = nepi_nex.setFactoryDriverOrder(nex_database)
    nex_database = activateAllDrivers(nex_database)
    rospy.set_param("~nex_database",nex_database)

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
      # Initialize nex_database
      print("****************************")
      print("remove next debug line")
      rospy.set_param("~nex_database",dict())
      nex_database = rospy.get_param("~nex_database",dict())
      print(nex_database)
      if len(nex_database.keys()) > 0:
        nex_database = nepi_nex.updateDriversDict(self.drivers_folder,nex_database)
      else:
        nex_database = nepi_nex.getDriverDict(self.drivers_folder)
        nex_database = nepi_nex.setFactoryDriverOrder(nex_database)
        [success,nex_database] = nepi_nex.activateAllDrivers(nex_database)
      rospy.set_param("~nex_database",nex_database)
      self.init_nex_database = nex_database
      self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
      rospy.set_param("~nex_database",self.init_nex_database)
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






