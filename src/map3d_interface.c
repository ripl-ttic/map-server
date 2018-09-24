#ifndef COMPILE_WITHOUT_MAP_SUPPORT
#endif
#include "map3d_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <zlib.h>

#include <carmen_utils/carmen.h>

#define DIV_PER_METER 10

int CMi, CMj;
static int map_type = 0;

//this looks like a very weird method 
ripl_gridmap_t * carmen3d_map_create_compressed_carmen3d_gridmap_t(float *complete_map, char * map_name, ripl_gridmap_t* gmappermap, double scale, double shift)
{
  ripl_gridmap_t * lcm_msg = (ripl_gridmap_t *) calloc(1,sizeof(ripl_gridmap_t));
  lcm_msg->config.x_size = gmappermap->config.x_size;
  lcm_msg->config.y_size = gmappermap->config.y_size;
  lcm_msg->config.resolution = gmappermap->config.resolution;
//  lcm_msg->config.y_size = config->y_size;
//  lcm_msg->config.resolution = config->resolution;
  lcm_msg->config.map_name = strdup(map_name);

  lcm_msg->center.x = gmappermap->center.x;
  lcm_msg->center.y = gmappermap->center.y;
  lcm_msg->type = 1;//CARMEN3D_MAP_GRID_MAP_TYPE_OCCUPANCY;
  lcm_msg->sizeX2 = gmappermap->sizeX2;
  lcm_msg->sizeY2 = gmappermap->sizeY2;
  lcm_msg->map_version = gmappermap->map_version;

  float *map_copy = (float*)malloc(lcm_msg->config.x_size*lcm_msg->config.y_size * sizeof(float));
  for (int i=0; i < lcm_msg->config.x_size * lcm_msg->config.y_size; i++) {
    map_copy[i] = scale * complete_map[i] + shift;
  }

  uLong uncompressed_size = lcm_msg->config.x_size * lcm_msg->config.y_size * sizeof(float);

  uLong compress_buf_size;
  int compress_return = -1;

  compress_buf_size = uncompressed_size * 1.1 + 12; //with extra space for zlib
  lcm_msg->map = (unsigned char *) malloc(compress_buf_size);
  carmen_test_alloc(lcm_msg->map);

  compress_return = compress2((Bytef *) lcm_msg->map,
      (uLong *) &compress_buf_size, (Bytef *) map_copy,
      (uLong) uncompressed_size, Z_BEST_SPEED);
  if (compress_return != Z_OK) {
    carmen_die("ERROR: Could not compress map!\n");
  }

  lcm_msg->size = compress_buf_size;
  lcm_msg->compressed = 1;
  lcm_msg->utime = bot_timestamp_now();

  free(map_copy);

  return lcm_msg;
}

carmen_inline carmen_map_p carmen3d_map_map3d_map_copy(ripl_map_p map)
{
  carmen_map_p new_map;
  int i;

  new_map = (carmen_map_p) calloc(1, sizeof(carmen_map_t));
  carmen_test_alloc(new_map);

  (*new_map).complete_map = (*map).complete_map;
  (*new_map).config = (*map).config;
  //	(*new_map).config.map_name = "testMap";
  (*new_map).map = (*map).map;
  new_map->complete_map = (float *) calloc(map->config.x_size * map->config.y_size, sizeof(float));
  carmen_test_alloc(new_map->complete_map);
  memcpy(new_map->complete_map, map->complete_map, sizeof(float) * map->config.x_size * map->config.y_size);

  new_map->map = (float **) calloc(map->config.x_size, sizeof(float *));
  carmen_test_alloc(new_map->map);

  for (i = 0; i < new_map->config.x_size; i++)
    new_map->map[i] = new_map->complete_map + i * new_map->config.y_size;

  return new_map;
}

int convert_and_publish_map(carmen_map_p map_p, lcm_t *lcm, const char *name){

  static ripl_gridmap_t lcm_msg;
  lcm_msg.config.x_size = map_p->config.x_size;
  lcm_msg.config.y_size = map_p->config.y_size;
  lcm_msg.config.resolution =  map_p->config.resolution;
  lcm_msg.config.map_name = strdup(name);

  lcm_msg.center.x = map_p->config.x_size/2* map_p->config.resolution;
  lcm_msg.center.y = map_p->config.y_size/2* map_p->config.resolution;
  
  lcm_msg.type = 1;//CARMEN3D_MAP_GRID_MAP_TYPE_OCCUPANCY;
  lcm_msg.sizeX2 = map_p->config.x_size;
  lcm_msg.sizeY2 = map_p->config.y_size;
  lcm_msg.map_version = 1;

  for (int i=0;i<map_p->config.x_size;i++){
    for (int j=0;j<map_p->config.y_size;j++){
        if (map_p->map[i][j]==-1)
            map_p->map[i][j] = MAP_UNEXPLORED_VALUE;
    }
  }
  
  ripl_gridmap_t *to_pub  = carmen3d_map_create_compressed_carmen3d_gridmap_t(map_p->complete_map, 
							      lcm_msg.config.map_name, &lcm_msg, 1, 0);
  
  //publish the map once when we load it  
  //ripl_gridmap_t_publish(lcm, GMAPPER_GRIDMAP_CHANNEL, to_pub);    
  ripl_gridmap_t_publish(lcm, "FINAL_SLAM", to_pub);  

  ripl_multi_gridmap_t lcm_full_map_msg;
  //message that holds the multi-floor map
  lcm_full_map_msg.no_floors = 1;
  lcm_full_map_msg.current_floor_ind = 0;  //this current floor is the index for the map 
  lcm_full_map_msg.maps = (ripl_floor_gridmap_t *)malloc(1* sizeof(ripl_floor_gridmap_t));
  memcpy(&lcm_full_map_msg.maps[0].gridmap, to_pub, sizeof(ripl_gridmap_t));    
  lcm_full_map_msg.maps[0].floor_no = 1;

  //for the multi-floor map   
  ripl_multi_gridmap_t_publish(lcm, "MULTI_FLOOR_MAPS", &lcm_full_map_msg);


  return 0;
}


void carmen3d_map_uncompress_lcm_map(ripl_map_t* new_map, const ripl_gridmap_t* map_msg)
{

  new_map->config.x_size = map_msg->config.x_size;
  new_map->config.y_size = map_msg->config.y_size;
  new_map->config.resolution = map_msg->config.resolution;
  //	new_map->config.map_name = strdup(map_msg->config.map_name);
  new_map->config.map_name = map_msg->config.map_name;
  new_map->complete_map = (float *) calloc(new_map->config.x_size * new_map->config.y_size, sizeof(float));
  carmen_test_alloc(new_map->complete_map);

#ifndef NO_ZLIB
  uLong uncompress_return;
  uLong uncompress_size;
  uLong uncompress_size_result;
  if (map_msg->compressed) {
    uncompress_size = new_map->config.x_size * new_map->config.y_size * sizeof(float);
    uncompress_size_result = uncompress_size;
    uLong compressed_size = map_msg->size;
    uncompress_return = uncompress((Bytef *) new_map->complete_map, (uLong *) &uncompress_size_result,
        (Bytef *) map_msg->map, (uLong) compressed_size);
    if (uncompress_return != Z_OK)
      carmen_die("ERROR uncompressing the map, ret = %lu", uncompress_return);

    new_map->map_zero.x = map_msg->center.x;
    new_map->map_zero.y = map_msg->center.y;

    new_map->midpt.x = new_map->config.x_size * new_map->config.resolution / 2;
    new_map->midpt.y = new_map->config.y_size * new_map->config.resolution / 2;
    new_map->sizeX2 = map_msg->sizeX2;
    new_map->sizeY2 = map_msg->sizeY2;

  }
  else {
    memcpy(new_map->complete_map, map_msg->map, map_msg->size);
  }
#else
  if (map_msg->compressed)
  {
    carmen_warn("Received compressed map from server. This program was\n"
        "compiled without zlib support, so this map cannot be\n"
        "used. Sorry.\n");

    memset(new_map, 0, sizeof(ripl_map_t));

    return;
  }
  else
  {
    memcpy(new_map->complete_map, map_msg->map, map_msg->size);
  }
#endif
  new_map->map = (float **) calloc(map_msg->config.x_size, sizeof(float *));
  carmen_test_alloc(new_map->map);
  for (int i = 0; i < map_msg->config.x_size; i++) {
    new_map->map[i] = new_map->complete_map + i * map_msg->config.y_size;
  }
}

//map co-ord - where the map left corner is 0 
carmen_point_t carmen3d_map_global_to_map_coordinates(carmen_point_t global_pt, ripl_map_t* map)
{
  carmen_point_t map_pt;
  map_pt.x = global_pt.x + map->midpt.x - map->map_zero.x;
  map_pt.y = global_pt.y + map->midpt.y - map->map_zero.y;
  map_pt.theta = global_pt.theta;// + M_PI/2;
  return map_pt;
}

//back to global co-ordinate set 
carmen_point_t carmen3d_map_map_to_global_coordinates(carmen_point_t map_pt, ripl_map_t* map)
{
  carmen_point_t global_pt;

  global_pt.x = map_pt.x - map->midpt.x + map->map_zero.x;
  global_pt.y = map_pt.y - map->midpt.y + map->map_zero.y;
  global_pt.theta = map_pt.theta;// - M_PI/2;
  return global_pt;
}

/**
 * TBD - move to map inferace (only here b/c it doesn't accept a map since we don't have a carmen_map_p and interface is not a cpp)
 */
void carmen3d_map3d_global_to_map_index_coordinates(carmen_point_t global_pt, carmen_point_t map_midpt,
    carmen_point_t map_zero, double map_resoln, int *map_idx_x, int *map_idx_y)
{
    
     *map_idx_x = carmen_round((global_pt.x + map_midpt.x - map_zero.x) / map_resoln);
     *map_idx_y = carmen_round((global_pt.y + map_midpt.y - map_zero.y) / map_resoln);
     
    //    *map_idx_x = floor((global_pt.x + map_midpt.x - map_zero.x) / map_resoln);
    //*map_idx_y = floor((global_pt.y + map_midpt.y - map_zero.y) / map_resoln);
}

void carmen3d_map3d_map_index_to_global_coordinates(double *global_pt_x, double *global_pt_y, carmen_point_t map_midpt,
    carmen_point_t map_zero, double map_resoln, int map_idx_x, int map_idx_y)
{
    //this is not the cell center - this is the left bottom corner of the cell 
    //    *global_pt_x = map_idx_x * map_resoln + map_zero.x - map_midpt.x;
    //*global_pt_y = map_idx_y * map_resoln + map_zero.y - map_midpt.y;
    *global_pt_x = map_idx_x * map_resoln + map_zero.x - map_midpt.x;// + map_resoln / 2.0;
    *global_pt_y = map_idx_y * map_resoln + map_zero.y - map_midpt.y;// + map_resoln / 2.0;
}

void carmen3d_map_initialize(ripl_map_p map)
{
  map->map_zero.x = 0;
  map->map_zero.y = 0;
  map->midpt.x = 0;
  map->midpt.y = 0;
}

void carmen3d_map_change_point_update(carmen_point_t* old_pt, carmen_point_t* new_pt, ripl_map_p old_map,
    ripl_map_p new_map)
{
  carmen_point_t temp_pt;
  temp_pt = carmen3d_map_map_to_global_coordinates(*old_pt, old_map);
  *new_pt = carmen3d_map_global_to_map_coordinates(temp_pt, new_map);
  new_pt->theta = 0;
}

carmen_inline void carmen3d_map_destroy(ripl_map_p *map)
{
  free((*map)->complete_map);
  free((*map)->map);
  free((*map));
  *map = NULL;
}

carmen_inline void carmen3d_map3d_destroy_map(carmen3d_map3d_grid_map_p map)
{
  for (int i = 0; i < map->numcells_x; i++) {
    for (int j = 0; j < map->numcells_y; j++) {
      free(map->map[i][j]);
    }
    free(map->map[i]);
  }
  free(map->map);
}


