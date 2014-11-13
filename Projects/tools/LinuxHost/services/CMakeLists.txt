cmake_minimum_required(VERSION 2.8.4)
project(RemoTI_Linux_services)

string(TOUPPER ${PROJECT_NAME} PROJECT_UPPER_NAME)
set(${PROJECT_UPPER_NAME}_INCLUDES ${PROJECT_SOURCE_DIR} CACHE STRING "${PROJECT_SOURCE_DIR} includes directory")
message(STATUS "Setting ${PROJECT_NAME} include path")

add_custom_target(${PROJECT_NAME})

install(FILES saddr.h DESTINATION includes)
