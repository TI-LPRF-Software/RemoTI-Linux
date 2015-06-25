cmake_minimum_required(VERSION 2.8)
project(RemoTI_server)

string(TOUPPER ${PROJECT_NAME} PROJECT_UPPER_NAME)
set(${PROJECT_UPPER_NAME}_INCLUDES ${PROJECT_SOURCE_DIR} CACHE STRING "${PROJECT_SOURCE_DIR} includes directory")
message(STATUS "Setting ${PROJECT_NAME} include path")

message(STATUS ${REMOTI_COMMON_INCLUDES})

include_directories(${REMOTI_COMMON_INCLUDES})

set(${PROJECT_NAME}_sources npi_lnx_ipc.c
  npi_lnx_uart.c
  npi_lnx_spi.c
  npi_lnx_i2c.c
  hal_gpio.c
  hal_i2c.c
  hal_spi.c
  hal_dbg_ifc.c
)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME} pthread RemoTI_application_common)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

set(REMOTI_CONF_PATH "/etc/RemoTI")

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES ${CMAKE_SOURCE_DIR}/RemoTI_RNP.cfg DESTINATION /etc/RemoTI)
install(FILES ${CMAKE_SOURCE_DIR}/configuration/systemd/remoti-linux.service DESTINATION /lib/systemd/system)
install(FILES ${SERVICE_FILE} DESTINATION /etc/systemd/system)
