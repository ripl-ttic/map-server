add_definitions(
#    -ggdb3
    -std=gnu99
)

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
)

add_executable(map-server mapserver.c)

pods_use_pkg_config_packages(map-server
    gthread-2.0
    lcm
    bot2-core
    bot2-frames
    lcmtypes_hr-lcmtypes
    lcmtypes_obstacle-detector
    lcmtypes_map
    lcmtypes_gridmap
    path-utils)

pods_install_executables(map-server)
