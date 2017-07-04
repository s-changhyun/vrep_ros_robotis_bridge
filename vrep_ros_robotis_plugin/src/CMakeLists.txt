
include_directories(${PROJECT_SOURCE_DIR}/include) 

set(SRCS
   v_repExtRosRobotisBridge.cpp
   RobotisApp.cpp
   VrepJointController.cpp
   ${VREP_ROOT_DIR}/programming/common/v_repLib.cpp
	)


add_library(v_repExtRosRobotisBridge SHARED ${SRCS})
target_link_libraries(v_repExtRosRobotisBridge ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(v_repExtRosRobotisBridge PROPERTIES COMPILE_FLAGS "-fopenmp")

#install(TARGETS v_repExtRosRobotisBridge
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(TARGETS v_repExtRosRobotisBridge LIBRARY DESTINATION ${VREP_ROOT_DIR})
#install(TARGETS v_repExtTeleKyb LIBRARY DESTINATION "./" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
