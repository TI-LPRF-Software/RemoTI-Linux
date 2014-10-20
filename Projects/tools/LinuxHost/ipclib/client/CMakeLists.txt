cmake_minimum_required(VERSION 2.8.4)
project(RemoTI_Linux_client)

string(TOUPPER ${PROJECT_NAME} PROJECT_UPPER_NAME)
set(${PROJECT_UPPER_NAME}_INCLUDES ${PROJECT_SOURCE_DIR} CACHE STRING "${PROJECT_SOURCE_DIR} includes directory")
message(STATUS "Setting ${PROJECT_NAME} include path")

message(STATUS ${REMOTI_LINUX_COMMON_INCLUDES})

include_directories(${PROJECT_NAME} ${REMOTI_LINUX_COMMON_INCLUDES} ${REMOTI_LINUX_SERVICES_INCLUDES})

set(${PROJECT_NAME}_sources npi_ipc_client.c npi_rti.c)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_sources})
add_dependencies(${PROJECT_NAME} RemoTI_Linux_common RemoTI_Linux_services)