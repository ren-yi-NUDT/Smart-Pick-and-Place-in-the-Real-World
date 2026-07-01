cmake_minimum_required(VERSION 2.8.3)

project(rm_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(PROGRAMS scripts/joint_state_relay.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
