cmake_minimum_required(VERSION 2.8.4)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include("witbe_rpm_utils")

# Configure
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -DRNP_HOST -DNPI_RTI -DMRDY_EARLY_FIX -DRNP_HOST -DNPI_SPI=TRUE -DNPI_UART=TRUE -DNPI_I2C=TRUE -DNPI_UART_USB=TRUE -D__DEBUG_TIME__ -D__DEBUG_TIME__I2C -DSRDY_INTERRUPT")
set(REMOTI_APPLICATION_COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/application/common CACHE STRING "${PROJECT_SOURCE_DIR} includes directory")

# Build
add_subdirectory(services)
add_subdirectory(ipclib)
add_subdirectory(application)

# Package
witbe_rpm_exclude_path_recursive("/etc/systemd/system")
witbe_rpm_exclude_path_recursive("${CMAKE_INSTALL_PREFIX}")
witbe_rpm_exclude_path("${CMAKE_INSTALL_PREFIX}/include")
witbe_rpm_exclude_path("${CMAKE_INSTALL_PREFIX}/bin")
witbe_rpm_exclude_path("${CMAKE_INSTALL_PREFIX}/lib")

set(CPACK_RPM_COMPONENT_INSTALL ON)
set(CPACK_GENERATOR RPM)
set(CPACK_SET_DESTDIR ON)
set(CPACK_PACKAGE_NAME remoti)
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_PACKAGE_CONTACT "Gauthier Lamaison")
set(CPACK_PACKAGE_VENDOR Witbe)
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_COMPONENT_devel_DEPENDS libs)
set(CPACK_RPM_devel_PACKAGE_REQUIRES remoti-libs)
set(CPACK_RPM_libs_PACKAGE_REQUIRES "")
set(CPACK_RPM_server_PACKAGE_REQUIRES "")

set(CPACK_RPM_devel_POST_INSTALL_SCRIPT_FILE "")
set(CPACK_RPM_libs_POST_INSTALL_SCRIPT_FILE "")
set(CPACK_RPM_server_POST_INSTALL_SCRIPT_FILE "${CMAKE_SOURCE_DIR}/cmake/rpm_server_post_install.sh")

set(CPACK_RPM_devel_POST_INSTALL_SCRIPT_FILE "")
set(CPACK_RPM_libs_POST_INSTALL_SCRIPT_FILE "")
set(CPACK_RPM_server_POST_UNINSTALL_SCRIPT_FILE "${CMAKE_SOURCE_DIR}/cmake/rpm_server_post_uninstall.sh")
include(CPack)
