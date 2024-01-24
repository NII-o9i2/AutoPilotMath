set(BUILDTOOLS_TARGET_PLATFORM "x86_64")

#######################
#    Project Config
#######################
set(PLATFORM_NAME x86_64)
add_definitions(-Dx86_64)

#######################
#    Install Config
#######################
set(CMAKE_INSTALL_PREFIX "/opt/senseauto/tmp/senseauto-decision-planning")
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
