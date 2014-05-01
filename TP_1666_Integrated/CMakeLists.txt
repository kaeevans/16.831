cmake_minimum_required (VERSION 2.6)


# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")


add_executable(TP_1106_Capture main.cpp testproject.cpp)


TARGET_LINK_LIBRARIES(TP_1106_Capture ${OpenCV_LIBS} "/usr/lib/libueye_api.so")



