set(BUILDTOOLS_TARGET_PLATFORM "mdc")
find_package(ADBuildTools REQUIRED)

#######################
#    Project Config
#######################
set(CMAKE_INSTALL_PREFIX "/opt/senseauto/tmp/senseauto-pilot-planning")
set(CMAKE_INSTALL_RPATH ".${CMAKE_INSTALL_PREFIX}/lib")

set(AD_CROSS_COMPILE ON)
set(MDC_ENABLED ON)
set(PLATFORM_NAME "aarch64_ub18_clang")

#set(BUILD_CYBER_IN_SENSETIME_DOCKER ON)
#######################
#    Install Config
#######################