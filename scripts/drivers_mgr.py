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
import rosnode
import sys
import subprocess
import time

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_nex

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_ros_interfaces.msg import DriversStatus, DriverStatus, DriverUpdateState, DriverUpdateOrder, DriverUpdateOption #, DriverUpdateMsg
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

class NepiDriversMgr(object):

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

  status_drivers_msg = DriversStatus()
  last_status_drivers_msg = DriversStatus()

  discovery_active_dict = dict()

  selected_driver = "None"
    

  #################################################################
  DEFAULT_NODE_NAME = "drivers_mgr"
  def publishMsg(self,msg, level = None, line_num = None):
    nepi_ros.publishMsg(self.node_name,self.msg_pub, msg, level, line_num)
  #######################
  ### Node Initialization
  def __init__(self):
    rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    rospy.loginfo("Starting " + self.DEFAULT_NODE_NAME)
    self.node_name = rospy.get_name().split('/')[-1]
    # Create a node msg publisher
    self.msg_pub = rospy.Publisher("~messages", String, queue_size=1)
    time.sleep(1)
    #################################################################

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
    self.drivers_status_pub = rospy.Publisher("~status", DriversStatus, queue_size=1, latch=True)
    self.driver_status_pub = rospy.Publisher("~status_driver", DriverStatus, queue_size=1, latch=True)

    # Setup message publisher and init param server
    self.publishMsg("Starting Initialization Processes")
    self.initParamServerValues(do_updates = False)
    self.resetParamServer(do_updates = False)

    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)
    self.save_cfg_if.saveConfig() # Save config after initialization for next time

 
    ## Mgr ROS Setup 
    mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
    self.initParamServerValues(do_updates=False)


    # Drivers Management Scubscirbers
    rospy.Subscriber('~select_driver', String, self.selectDriverCb)
    rospy.Subscriber('~update_state', DriverUpdateState, self.updateStateCb)
    rospy.Subscriber('~update_order', DriverUpdateOrder, self.updateOrderCb)
    rospy.Subscriber('~update_option', DriverUpdateOption, self.updateOptionCb)
    #rospy.Subscriber('~select_driver_status', DriverUpdateMsg, self.updateMsgCb)

    rospy.Subscriber('~install_driver_pkg', String, self.installDriverPkgCb)
    rospy.Subscriber('~remove_driver', String, self.removeDriverCb)

    # Setup a driver folder timed check
    rospy.Timer(rospy.Duration(self.DRIVER_UPDATE_CHECK_INTERVAL), self.checkAndUpdateCb)
    time.sleep(1)
    ## Publish Status
    self.publish_status()

    #########################################################
    ## Initiation Complete
    self.publishMsg("Initialization Complete")
    #Set up node shutdown
    rospy.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    rospy.spin()
    #########################################################

        
  # ln = sys._getframe().f_lineno ; self.printND('Info',ln)
  def printND(self, level = 'Info',line_num = None):
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    self.publishMsg('',level,line_num)
    self.publishMsg('*******************')
    self.publishMsg('Printing Nex Driver Dictionary')
    for nex_name in nex_database.keys():
      nex_dict = nex_database[nex_name]
      self.publishMsg('',level,line_num)
      self.publishMsg(nex_name)
      self.publishMsg(str(nex_dict))

  def publish_status(self):
    self.publish_drivers_status()
    self.publish_driver_status()


  def publish_drivers_status(self):
    self.last_status_drivers_msg = self.status_drivers_msg
    self.status_drivers_msg = self.getDriversStatusMsg()
    if not rospy.is_shutdown():
      self.drivers_status_pub.publish(self.status_drivers_msg)

  def getDriversStatusMsg(self):
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    status_drivers_msg = DriversStatus()
    status_drivers_msg.drivers_path = self.drivers_folder
    status_drivers_msg.drivers_ordered_list = str(self.drivers_ordered_list)
    status_drivers_msg.drivers_active_path = self.drivers_active_folder
    status_drivers_msg.drivers_active_list = str(self.drivers_active_list)
    status_drivers_msg.drivers_install_path = self.drivers_install_folder
    status_drivers_msg.drivers_install_list = str(self.drivers_install_files)
    status_drivers_msg.selected_driver = self.selected_driver
    return status_drivers_msg

  
  def publish_driver_status(self):
    self.status_driver_msg = self.getDriverStatusMsg()
    if not rospy.is_shutdown():
      self.driver_status_pub.publish(self.status_driver_msg)


  def getDriverStatusMsg(self):
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    driver_name = self.selected_driver
    status_driver_msg = DriverStatus()
    if driver_name in nex_database.keys():
      driver = nex_database[driver_name]
      status_driver_msg.name = driver_name
      status_driver_msg.group = driver['group']
      status_driver_msg.group_id  = driver['group_id']
      status_driver_msg.interfaces  = str(driver['driver_interfaces'])
      status_driver_msg.options  = str(driver['driver_options'])
      status_driver_msg.set_option  = driver['driver_set_option']
      status_driver_msg.discovery = driver['discovery_method']
      status_driver_msg.other_users_list  = str(driver['users'])
      status_driver_msg.active_state  = driver['active']
      status_driver_msg.order  = driver['order']
      status_driver_msg.msg_str = driver['msg']
    return status_driver_msg

  
  def checkAndUpdateCb(self,timer):
    ## First update Database
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
        nex_database = nepi_nex.getDriversgetDriversDict(self.drivers_folder)
        nex_database = nepi_nex.setFactoryDriverOrder(nex_database)
        nex_database = activateAllDrivers(nex_database)
        ln = sys._getframe().f_lineno ; self.printND('Error',ln)
    rospy.set_param("~nex_database",nex_database)
    self.drivers_ordered_list = nepi_nex.getDriversOrderedList(nex_database)
    self.drivers_active_list = nepi_nex.getDriversActiveOrderedList(nex_database)
    ## Next process active driver processes
    '''
    # Get list of running nodes
    node_list = nepi_ros.get_node_list()
      for i in range(len(node_list)):
        node_list[i] = node_list.split("/")[-1]
    ln = sys._getframe().f_lineno ; self.publishMsg("Node_list: " + str(node_list))
    # Check and process active driver list entries:
    for nex_name in self.drivers_active_list
      nex_dict = nex_database[nex_name]
      discovery_name = nex_dict['discovery_name']
      discovery_file = nex_dict['discovery_file_name']
      discovery_path = nex_dict['discovery_file_path']
      discovery_module = nex_dict['discovery_module_name']
      discovery_class = nex_dict['discovery_class_name']
      discovery_methed = nex_dict['discovery_method']
      discovery_process = nex_dict['discovery_process']
      discovery_ids = nex_dict['discovery_ids']
      discovery_ignore_ids = nex_dict['discovery_ignore_ids']

      driver_name = nex_dict['driver_name']
      driver_file = nex_dict['driver_file_name']
      driver_path = nex_dict['driver_file_path']
      driver_module = nex_dict['driver_module_name']
      driver_class = nex_dict['driver_class_name']
      driver_interfaces = nex_dict['driver_interfaces']
      driver_options = nex_dict['driver_options']
      driver_option = nex_dict['driver_set_option']
      ###############################
      # Check Auto-Node processes
      if discovery_method == 'AUTO' and discovery_process = "LAUNCH":
        discovery_node_name = discovery_name.lower() + "_discovery"
        if discovery_node_name not in node_list:
          #Setup required param server nex_dict for discovery node
          dict_param_name = discovery_node_name + "/nex_dict"
          rospy.set_param(dict_param_name,nex_dict)
          #Try and launch node
          [success, msg, subprocess] = nepi_nex.LaunchDriverNode(discovery_file, discovery_path, discovery_node_name)
          if success:
            nex_database[nex_name]['msg'] = "Discovery process lanched"
            discovery_active_dict[nex_name]['process'] = "LAUNCH"
            discovery_active_dict[nex_name]['node_name'] = discovery_node_name
            discovery_active_dict[nex_name]['subprocess'] = subprocess
          else:
            nex_database[nex_name]['msg'] = msg
        else:
          nex_database[nex_name]['msg'] = "Discovery process running"

      # Run Auto-Run processes 
      if discovery_method == 'AUTO' and discovery_process = "RUN":
        pass # ToDo

      # Call Auto-Call processes 
      if discovery_method == 'AUTO' and discovery_process = "CALL":
        pass # ToDo

      # Do Manual processes 
      if discovery_method == 'MANUAL':
        if 'SERIAL' in driver_interfaces:
          pass # ToDo
        if 'SERIALUSB' in driver_interfaces:
          pass # ToDo
        if 'USB' in driver_interfaces:
          pass # ToDo
        if 'IP' in driver_interfaces:
          pass # ToDo
      time.sleep(1)
      ################################    
      ## Check and purge disabled driver proccess that might be running
      # First check on running nodes
      node_list = nepi_ros.get_node_list()
      for i in range(len(node_list)):
        node_list[i] = node_list.split("/")[-1]
      purge_list = []
      for nex_name in self.drivers_ordered_list:
        if nex_name not in self.drivers_active_list and nex_name in self.discovery_active_dict.keys():
            discovery_active_dict[nex_name]['process'] = "LAUNCH"
            subprocess = discovery_active_dict[nex_name]['subprocess']
            if discovery_process == "LAUNCH":
                if node_name in node_list:
                  discovery_node_name = discovery_active_dict[nex_name]['node_name']
                  [kill_list,fail_list] = rosnode.kill_nodes([discovery_node_name])
          
            time.sleep(2)    
            # Next check running processes
            if subprocess.poll() is not None:
              subprocess.kill()
              # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
              # rosnode cleanup won't find the disconnected node until the process is fully terminated
              try:
                subprocess.wait(timeout=1)
              except:
                pass
              purge_list.append(nex_name)

              ###############
              ## Add process kill
        # purge from active discovery dict
        for nex_name in purge_list:
          if nex_name in self.discovery_active_dict.keys():
            del self.discovery_active_dict[nex_name]
      '''
    # Publish Status
    self.publish_status()


    

    
  

  ###################
  ## Drivers Mgr Callbacks

  def selectDriverCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.data
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      self.selected_driver = driver_name
    self.publish_status()

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
    
  def updateMsgCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.driver_name
    msg_data = msg.data
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      nex_database[driver_name]['msg'] = msg_data
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()

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






   #######################
  ### Mgr Config Functions

 

  def resetMgrCb(self,msg):
    self.resetMgr()

  def resetMgr(self):
    # reset drivers dict
    nex_database = nepi_nex.getDriversgetDriversDict(self.drivers_folder)
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
      #print("****************************")
      #print("remove next debug line")
      #rospy.set_param("~nex_database",dict())
      nex_database = rospy.get_param("~nex_database",dict())
      if len(nex_database.keys()) > 0:
        nex_database = nepi_nex.updateDriversDict(self.drivers_folder,nex_database)
      else:
        nex_database = nepi_nex.getDriversDict(self.drivers_folder)
        nex_database = nepi_nex.setFactoryDriverOrder(nex_database)
        [success,nex_database] = nepi_nex.activateAllDrivers(nex_database)
      rospy.set_param("~nex_database",nex_database)
      self.init_nex_database = nex_database
      self.resetParamServer(do_updates)

      ln = sys._getframe().f_lineno ; self.printND('Error',ln)

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
  NepiDriversMgr()







