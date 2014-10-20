cmake_minimum_required(VERSION 2.8.4)
project(RemoTI_Linux)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DRNP_HOST -DNPI_RTI")

add_subdirectory(services)
add_subdirectory(ipclib)