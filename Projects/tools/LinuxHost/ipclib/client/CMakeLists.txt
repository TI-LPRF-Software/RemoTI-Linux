cmake_minimum_required(VERSION 2.8)
project(RemoTI_client)

string(TOUPPER ${PROJECT_NAME} PROJECT_UPPER_NAME)
set(${PROJECT_UPPER_NAME}_INCLUDES ${PROJECT_SOURCE_DIR} CACHE STRING "${PROJECT_SOURCE_DIR} includes directory")
message(STATUS "Setting ${PROJECT_NAME} include path")

include_directories(${REMOTI_COMMON_INCLUDES} ${REMOTI_SERVICES_INCLUDES} ${REMOTI_APPLICATION_COMMON_INCLUDES})

set(${PROJECT_NAME}_sources npi_ipc_client.c npi_rti.c)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME} RemoTI_application_common)
add_dependencies(${PROJECT_NAME} RemoTI_common RemoTI_services RemoTI_application_common)

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES npi_ipc_client.h rti_lnx.h rti_lnx_constants.h DESTINATION include/RemoTI)
