// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.5 on October 26th 2013

#include "vrep_ros_robotis_plugin/v_repExtRosRobotisBridge.h"
#include <iostream>
#include "v_repLib.h"

#include <time.h>

#include "vrep_ros_robotis_plugin/RobotisApp.h"

#include <ros/ros.h>
#include <boost/bind.hpp>

#ifdef _WIN32
#include <shlwapi.h> // required for PathRemoveFileSpec function
#define WIN_AFX_MANAGE_STATE AFX_MANAGE_STATE(AfxGetStaticModuleState())
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
#define WIN_AFX_MANAGE_STATE
#endif /* __linux || __APPLE__ */

LIBRARY vrepLib;

RobotisApp *robotis_app = new RobotisApp();

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
  // Dynamically load and bind V-REP functions:
  // ******************************************
  // 1. Figure out this plugin's directory:
  char curDirAndFile[1024];
  getcwd(curDirAndFile, sizeof(curDirAndFile));

  std::string currentDirAndPath(curDirAndFile);

  // 2. Append the V-REP library's name:
  std::string temp(currentDirAndPath);
  temp+="/libv_rep.so";

  // 3. Load the V-REP library:
  vrepLib=loadVrepLibrary(temp.c_str());
  if (vrepLib==NULL)
  {
    std::cout << "Error, could not find or correctly load v_rep.dll. Cannot start 'UR10Rob' plugin.\n";
    return(0); // Means error, V-REP will unload this plugin
  }
  if (getVrepProcAddresses(vrepLib)==0)
  {
    std::cout << "Error, could not find all required functions in v_rep.dll. Cannot start 'UR10Rob' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return(0); // Means error, V-REP will unload this plugin
  }

  // Check the V-REP version:
  int vrepVer;
  simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
  if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
  {
    std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return(0); // Means error, V-REP will unload this plugin
  }

  //Initialize ROS
  if(!ros::isInitialized())
  {
    int argc = 0;
    ros::init(argc,NULL,"vrep");
  }

  if(!ros::master::check())
  {
    printf("[ROS master is not running. Cannot start plugin\n");
    return (0);
  }

  printf("[v_repExtRosRobotisBridge] -> v_repStart()\n");

  return(3);
}

VREP_DLLEXPORT void v_repEnd()
{
  printf("[v_repExtRosRobotisBridge] -> v_repStop()\n");

  // This is called just once, at the end of V-REP
  unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
  // This is called quite often. Just watch out for messages/events you want to handle
  // This function should not generate any error messages:
  int errorModeSaved;
  simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
  simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
  void* retVal=NULL;

  if(message==sim_message_eventcallback_modulehandle)
  {
    // Timer Event
    robotis_app->process();
  }
  else if(message == sim_message_eventcallback_simulationabouttostart)
  {
    printf("[v_repExtRosRobotisBridge] -> Start Simulation\n");

    //	init code
    robotis_app->initialize();
  }
  else if(message==sim_message_eventcallback_simulationended)
  {
    printf("[v_repExtRosRobotisBridge] -> Stop Simulation\n");

    //	close code
    robotis_app->stop();
  }

  simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings

  return(retVal);
}
