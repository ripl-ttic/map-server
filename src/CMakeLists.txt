add_definitions(
#    -ggdb3
    -std=gnu99
    )

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
    )

pods_install_headers(map3d_interface.h map3d.h DESTINATION interfaces)

set(map3d_interface_sources
    map3d_interface.c
    )

add_library(map3d_interface SHARED ${map3d_interface_sources})
pods_install_libraries(map3d_interface)

pods_use_pkg_config_packages(map3d_interface
    lcm
    bot2-core
    lcmtypes_hr-lcmtypes
    lcmtypes_obstacle-detector
    lcmtypes_gridmap)

target_link_libraries(map3d_interface
  -lcarmen-utils
  -lz
)

pods_install_pkg_config_file(map3d_interface
    CFLAGS
    LIBS -lmap3d_interface
    REQUIRES
    VERSION 0.0.1)


set(mapserver_sources
  mapserver.c
  )

add_executable(mapserver ${mapserver_sources})

pods_use_pkg_config_packages(mapserver
    gthread-2.0
    lcm
    bot2-core
    bot2-frames
    lcmtypes_hr-lcmtypes
    lcmtypes_obstacle-detector
    lcmtypes_map
    lcmtypes_gridmap
    path-utils)

pods_install_executables(mapserver)
