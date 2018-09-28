/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

/** @addtogroup simulator libsimulator_interface **/
// @{

/** \file simulator_interface.h
 * \brief Definition of the interface of the module simulator.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef MAP3D_INTERFACE_H
#define MAP3D_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <bot_core/bot_core.h>
#include <carmen_utils/global.h>
#include <carmen_utils/map.h>
#include <carmen_utils/map_messages.h>
#include "map3d.h"

#include <lcmtypes/gridmap_lcmtypes.h>

#define MAP_EPS 0.05
#define MAP_FREE_VALUE 0
#define MAP_UNEXPLORED_VALUE 0.5
#define MAP_OBSTACLE_VALUE 1
#define MAP_UNTRAVERSABLE_RANGE 0.9
#define MAP_CAMERA_VIEWED_VALUE 10
#define MAP_CAMERA_FRONTIER_VALUE 15
#define MAP_WINDOW_VALUE 20
#define MAP_OUTSIDE_VALUE 30

#define MAP3D_FREE_SPACE_MIN 10
#define MAP3D_OCCUPIED_SPACE_MAX 20
#define MAP3D_FREE_CELL_LASER_INC -0.1
#define MAP3D_FREE_CELL_STEREO_INC -0.01

    /*typedef struct {
      carmen3d_point_t vert1;
      carmen3d_point_t vert2;
      } carmen3d_map_edge_t;

      typedef struct {
      carmen_point_t min;
      carmen_point_t max;
      int numedges;
      carmen3d_map_edge_t* edges;
      } carmen3d_map_2dpolygon_t;
    */



carmen_map_p carmen3d_map_map3d_map_copy(ripl_map_p map);

int convert_and_publish_map(carmen_map_p map_p, lcm_t *lcm, const char *name);

void carmen3d_map_uncompress_lcm_map(ripl_map_t* uncompressmap, const ripl_gridmap_t* compressmsg);

carmen_point_t carmen3d_map_global_to_map_coordinates(carmen_point_t global_pt, ripl_map_t* map);

carmen_point_t carmen3d_map_map_to_global_coordinates(carmen_point_t map_pt, ripl_map_t* map);

void carmen3d_map3d_global_to_map_index_coordinates(carmen_point_t global_pt, carmen_point_t map_midpt, carmen_point_t map_zero, double map_resoln, int *map_idx_x, int *map_idx_y);

void carmen3d_map3d_map_index_to_global_coordinates(double *global_pt_x, double *global_pt_y, carmen_point_t map_midpt, carmen_point_t map_zero, double map_resoln, int map_idx_x, int map_idx_y);

void carmen3d_map_initialize(ripl_map_p map);

void carmen3d_map_change_point_update(carmen_point_t* old_pt, carmen_point_t* new_pt, ripl_map_p old_map,
                                          ripl_map_p new_map);
void carmen3d_map_destroy(ripl_map_p *map);

ripl_gridmap_t * carmen3d_map_create_compressed_carmen3d_gridmap_t(float *complete_map, char * map_name, ripl_gridmap_t* gmappermap, double scale, double shift);

static inline ripl_gridmap_t *carmen3d_map_create_compressed_carmen3d_gridmap_t_from_double(double *complete_map,
                                                                                char * map_name, ripl_gridmap_t* gmappermap, double scale, double shift) {
    int map_size = gmappermap->config.x_size * gmappermap->config.y_size ;
    float * float_map = (float * ) malloc(map_size* sizeof(float));
    for (int i=0;i<map_size;i++)
        float_map[i]=complete_map[i];
    ripl_gridmap_t * ret = carmen3d_map_create_compressed_carmen3d_gridmap_t(float_map, map_name, gmappermap, scale, shift);
    free(float_map);
    return ret;
}


#ifdef __cplusplus
}
#endif

#endif
// @}
