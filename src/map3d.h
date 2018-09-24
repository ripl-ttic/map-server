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

#ifndef CARMEN3D_MAP3D_H
#define CARMEN3D_MAP3D_H

#include <carmen_utils/global.h>
#include <carmen_utils/map.h>
#include <geom_utils/geometry.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct{
        int idx_x;
        int idx_y;
        carmen_point_t global_pt;
        carmen_point_t map_pt;
        double su_value;
        double weight;
    }carmen3d_map_frontier_t, *carmen3d_map_frontier_p;

    typedef struct{
        int num_frontiers;
        int frontiers_found;
        int clusters_found;
        carmen3d_map_frontier_t* frontiers;
    }carmen3d_map_frontier_list_t, *carmen3d_map_frontier_list_p;

    typedef struct {
        carmen_map_config_t config;
        float* complete_map;
        float** map;
        carmen_point_t map_zero;
        carmen_point_t midpt;
        int sizeX2;
        int sizeY2;
    } ripl_map_t, *ripl_map_p;

    typedef struct {
        float*** map;
        point3d_t map_origin;
        double resolution;
        int numcells_x;
        int numcells_y;
        int numcells_z;
    } carmen3d_map3d_grid_map_t, *carmen3d_map3d_grid_map_p;


#ifdef __cplusplus
}
#endif

#endif
