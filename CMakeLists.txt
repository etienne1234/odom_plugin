cmake_minimum_required(VERSION 2.8.3)
project(odom_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rqt_gui
	rqt_gui_cpp
	nav_msgs
)

find_path(QWT_INCLUDE_DIR qwt.h HINTS /usr/local/qwt-6.1.2/include)
find_library(QWT_LIBRARY qwt /usr/local/qwt-6.1.2/lib/)

set(INCLUDE_DIR "include/")
#set(QWT_INCLUDE_DIR "../../../../../../usr/local/qwt-6.1.2/include/")
#set(QWT_LIBRARIES "../../../../../../usr/local/qwt-6.1.2/lib/")

## System dependencies are found with CMake's conventions
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

 add_definitions(${Qt5Widgets_DEFINITIONS})
 set(CMAKE_CXX_FLAGS "${Qt5Core_EXECUTABLE_COMPILE_FLAGS}")
 set(CMAKE_CXX_FLAGS "${Qt5Gui_EXECUTABLE_COMPILE_FLAGS}")
 set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS ${QWT_INCLUDE_DIR} ${INCLUDE_DIR}
  LIBRARIES odom_plugin
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp nav_msgs 
  #  DEPENDS system_lib
)

catkin_python_setup()

qt5_wrap_cpp(odom_plugin_MOCS "include/odom_plugin.h")

###########
## Build ##
###########
include_directories( 
	${catkin_INCLUDE_DIRS}
	"include/"
	${Qt5Core_INCLUDES}
	${Qt5Widgets_INCLUDES} 
	${Qt5Gui_INCLUDES}
	${QWT_INCLUDE_DIR}
)

## Declare a C++ library
 add_library(odom_plugin
   src/odom_plugin.cpp
   include/odom_plugin.h
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(odom_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(odom_plugin_node src/odom_plugin_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(odom_plugin_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(odom_plugin
   Qt5::Core
   Qt5::Gui
   Qt5::Widgets
   ${catkin_LIBRARIES}
   ${QWT_LIBRARIES}
 )
