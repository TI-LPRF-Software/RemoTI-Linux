cmake_minimum_required(VERSION 2.8)
project(RemoTI_application_common)

string(TOUPPER ${PROJECT_NAME} PROJECT_UPPER_NAME)
set(${PROJECT_UPPER_NAME}_INCLUDES ${PROJECT_SOURCE_DIR} CACHE STRING "${PROJECT_SOURCE_DIR} includes directory")
message(STATUS "Setting ${PROJECT_NAME} include path")

include_directories(${REMOTI_COMMON_INCLUDES})

set(${PROJECT_NAME}_sources time_printf.c)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources})
