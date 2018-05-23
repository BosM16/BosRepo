## Collect Sources and make an executable

# Create a variable called helloworld_SOURCES containing all .cpp files:
# set(SOURCES EMC2.cpp main.cpp)
 set(SOURCES main.cpp perception.cpp helper.hpp monitors.cpp state_machine.cpp control.cpp)

# For a large number of source files you can create it in a simpler way
# using file() function:
#file(GLOB SOURCES *.cpp)

# Create an executable file called helloworld from sources:
add_executable(${executable_name} ${SOURCES})

## Collect Headers: headers in src
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(
    ${PROJECT_BINARY_DIR} # version number variables
    ${PROJECT_SOURCE_DIR}/src
    )
include_directories("/usr/include/eigen3")

## Collect Headers: external libraries + Link library (.so) to executable
if(OPENCV_FOUND)
     include_directories(${OpenCV_INCLUDE_DIRS})
     set(LIBS ${LIBS} ${OpenCV_LIBS})
endif(OPENCV_FOUND)
#
#target_link_libraries(${executable_name} ${LIBS})
target_link_libraries(${executable_name} ${LIBS} emc-framework)
