cmake_minimum_required(VERSION 2.8.4)
project(RemoTI_Linux_client)

string(TOUPPER ${PROJECT_NAME} PROJECT_UPPER_NAME)
set(${PROJECT_UPPER_NAME}_INCLUDES ${PROJECT_SOURCE_DIR} CACHE STRING "${PROJECT_SOURCE_DIR} includes directory")
message(STATUS "Setting ${PROJECT_NAME} include path")

include_directories(${REMOTI_LINUX_COMMON_INCLUDES} ${REMOTI_LINUX_SERVICES_INCLUDES} ${REMOTI_LINUX_APPLICATION_COMMON_INCLUDES})

set(${PROJECT_NAME}_sources npi_ipc_client.c npi_rti.c)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME} RemoTI_Linux_application_common)
add_dependencies(${PROJECT_NAME} RemoTI_Linux_common RemoTI_Linux_services RemoTI_Linux_application_common)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES npi_ipc_client.h rti_lnx.h rti_lnx_constants.h DESTINATION includes)
