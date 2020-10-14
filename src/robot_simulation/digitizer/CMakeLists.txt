cmake_minimum_required(VERSION 2.8.3)

project(digitizer)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros)

catkin_package()

find_package(roslaunch)

install(DIRECTORY launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

foreach(dir launch urdf models)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_install_python(PROGRAMS submissions/Suyash/PathFinder.py submissions/Suyash/PathFinderNode.py submissions/Suyash/Movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
