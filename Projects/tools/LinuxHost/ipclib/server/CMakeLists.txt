cmake_minimum_required(VERSION 2.8.4)
project(RemoTI_Linux_server)

string(TOUPPER ${PROJECT_NAME} PROJECT_UPPER_NAME)
set(${PROJECT_UPPER_NAME}_INCLUDES ${PROJECT_SOURCE_DIR} CACHE STRING "${PROJECT_SOURCE_DIR} includes directory")
message(STATUS "Setting ${PROJECT_NAME} include path")

message(STATUS ${REMOTI_LINUX_COMMON_INCLUDES})

include_directories(${REMOTI_LINUX_COMMON_INCLUDES})

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
target_link_libraries(${PROJECT_NAME} pthread)

execute_process(COMMAND uname -m OUTPUT_VARIABLE architecture OUTPUT_STRIP_TRAILING_WHITESPACE)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME NPI_lnx_${architecture}_server)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
