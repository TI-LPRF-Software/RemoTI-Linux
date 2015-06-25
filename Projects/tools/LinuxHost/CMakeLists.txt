cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -DRNP_HOST -DNPI_RTI -DMRDY_EARLY_FIX -DRNP_HOST -DNPI_SPI=TRUE -DNPI_UART=TRUE -DNPI_I2C=TRUE -DNPI_UART_USB=TRUE -D__DEBUG_TIME__ -D__DEBUG_TIME__I2C -DSRDY_INTERRUPT")
set(REMOTI_APPLICATION_COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/application/common CACHE STRING "${PROJECT_SOURCE_DIR} includes directory")

# Build
add_subdirectory(services)
add_subdirectory(ipclib)
add_subdirectory(application)
