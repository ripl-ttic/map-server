/****************************************
 ** Map Server for carmen3D tourguide
 ** Takes map data file and creates messages ready to be sent out
 ** Sachi Hemachandra
 ********************************************************/

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <getopt.h>
#include <glib.h>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <path_utils/path_util.h>

#include <lcmtypes/gridmap_lcmtypes.h>
#include <lcmtypes/hr_lcmtypes.h>
#include <lcmtypes/obstacle_detector_lcmtypes.h>
#include <lcmtypes/map_lcmtypes.h>

#define DIV_PER_METER 10
#define PUBLISH_RATE 1

typedef struct
{
    lcm_t *lcm;
    BotFrames *frames;
    lcm_eventlog_t *read_log;
    lcm_eventlog_t *write_log;
    int mode;
    gmlcm_multi_gridmap_t *multi_msg;
    maplcm_tagged_node_list_t *tagged_places;
    gmlcm_gridmap_t *single_map;
    maplcm_elevator_node_list_t *elevator_list;
    maplcm_topology_t *topology_list;
    obs_rect_list_t *sim_rects;
    //gmlcm_gridmap_t **multi_maps;
    int verbose;
    int preloaded_m_map;
    int slam_m_map;
    int robot_mode; //this is the mode of the robot - currently tourguide - 0, navigation - 1
    bot_core_pose_t *latest_pose;
    //int current_floor_ind; -lets not use this
    int current_floor_no;
    int multi_map_count;
    int save;
    int latest;

    guint publish_timer_id;
} state_t;

int send_sim_rects_for_floor(state_t *s){
    if(s->sim_rects !=NULL){
        //maplcm_topology_t_publish(s->lcm,"MAP_SERVER_TOPOLOGY", s->topology_list);
        //send for just the correct floor
        obs_rect_list_t msg;

        msg.utime = bot_timestamp_now();


        int count = 0;
        for(int i=0;i< s->sim_rects->num_rects; i++){
            if(s->sim_rects->rects[i].floor_no != s->current_floor_no){
                continue;
            }
            count++;
        }

        msg.num_rects = count;

        msg.rects = (obs_rect_t*) calloc(msg.num_rects, sizeof(obs_rect_t));

        count = 0;

        for(int i=0;i< s->sim_rects->num_rects; i++){
            if(s->sim_rects->rects[i].floor_no != s->current_floor_no){
                continue;
            }
            memcpy(&msg.rects[count], &(s->sim_rects->rects[i]), sizeof(obs_rect_t));
            count++;
        }

        if (s->verbose)
            fprintf(stderr,"Sent Sim rects\n");

        obs_rect_list_t_publish(s->lcm, "MAP_SERVER_RECTS", &msg);
        free(msg.rects);

        return 0;
    }
    return 1;
}


int get_floor_ind(state_t *s, int req_floor_no){
    if(s->multi_msg !=NULL){
        int no_floors = s->multi_msg->no_floors;
        for(int i=0; i< no_floors; i++){
            if(s->multi_msg->maps[i].floor_no == req_floor_no){
                return i;
            }
        }
    }
    return -1;
}

gmlcm_floor_gridmap_t* get_floor_map(state_t *s, int req_floor_no){
    if(s->multi_msg !=NULL){
        int no_floors = s->multi_msg->no_floors;
        for(int i=0; i< no_floors; i++){
            if(s->multi_msg->maps[i].floor_no == req_floor_no){
                return &s->multi_msg->maps[i];
            }
        }
    }
    return NULL;
}

gmlcm_gridmap_t* get_gridmap(state_t *s, int req_floor_no){
    if(s->multi_msg !=NULL){
        int no_floors = s->multi_msg->no_floors;
        for(int i=0; i< no_floors; i++){
            if(s->multi_msg->maps[i].floor_no == req_floor_no){
                return &s->multi_msg->maps[i].gridmap;
            }
        }
    }
    return NULL;
}

//send the floor map
void send_lcm_floor_map(char *requesting_prog, int floor_no, state_t *s){
    if(s->multi_msg !=NULL){
        gmlcm_floor_gridmap_t *fl_map = get_floor_map(s,floor_no);
        if(fl_map !=NULL){
            gmlcm_floor_gridmap_t_publish(s->lcm, "FMAP_SERVER", fl_map);
            send_sim_rects_for_floor(s);
        }
        else{
            fprintf(stderr, "Error - Could not find map for the floor\n");
            //we should send an error message
        }
    }
}

//send the floor map
void send_specific_lcm_floor_map(char *requesting_prog, int floor_no, state_t *s){
    if(s->multi_msg !=NULL){
        gmlcm_floor_gridmap_t *fl_map = get_floor_map(s,floor_no);
        if(fl_map !=NULL){
            gmlcm_floor_gridmap_t_publish(s->lcm, "SFMAP_SERVER", fl_map);
        }
        else{
            fprintf(stderr, "Error - Could not find map for the floor\n");
            //we should send an error message
        }
    }
}

void send_lcm_map(char *requesting_prog, state_t *s, int floor_no){
    if(s->multi_msg !=NULL){
        gmlcm_gridmap_t *gmap = get_gridmap(s,floor_no);
        if(gmap !=NULL){
            gmlcm_gridmap_t_publish(s->lcm, "MAP_SERVER", gmap);
            send_sim_rects_for_floor(s);
        }
        else{
            fprintf(stderr, "Error - Could not find map for the floor\n");
            //we should send an error message
        }
    }
}

//message handlers
void gridmap_handler(const lcm_recv_buf_t *rbuf, const char *channel, const gmlcm_gridmap_t *msg, void *user)
{
    state_t *s = (state_t *) user;
    /*
      Not handling single maps - right now
    */
}

void place_update_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const maplcm_place_node_t *msg, void *user)
{
    state_t *s = (state_t *) user;
    fprintf(stderr, "Received tagged place\n");
    if(s->topology_list == NULL){
        s->topology_list = (maplcm_topology_t *)calloc(1, sizeof(maplcm_topology_t));
        s->topology_list->place_list.place_count = 1;

        s->topology_list->place_list.trajectory = (maplcm_place_node_t *) calloc(1, sizeof(maplcm_place_node_t));
        //not sure if this is the way to copy it
        memcpy(&s->topology_list->place_list.trajectory[0], msg, sizeof(maplcm_place_node_t));
        //s->tagged_places->places[0]
    }
    else{
        s->topology_list->place_list.trajectory = (maplcm_place_node_t *) realloc(s->topology_list->place_list.trajectory, (s->topology_list->place_list.place_count + 1) * sizeof(maplcm_place_node_t));
        memcpy(&s->topology_list->place_list.trajectory[s->topology_list->place_list.place_count], msg, sizeof(maplcm_place_node_t));
        s->topology_list->place_list.place_count++;

        fprintf (stdout, "place count = %d\n", s->topology_list->place_list.place_count);
        fprintf(stderr,"Place 0: \t%s\n", s->topology_list->place_list.trajectory[0].name);
        for(int i=0; i < s->topology_list->place_list.place_count; i++){
            fprintf(stderr,"\t%s\n", s->topology_list->place_list.trajectory[i].name);
        }
    }

    for(int i=0; i < s->topology_list->place_list.place_count; i++){
        fprintf(stderr,"\t%s\n", s->topology_list->place_list.trajectory[i].name);
    }

    if(s->topology_list)
        maplcm_topology_t_publish(s->lcm, "TOPOLOGY" , s->topology_list);
}

void portal_update_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                           const maplcm_portal_node_t *msg, void *user)
{
    state_t *s = (state_t *) user;
    fprintf(stderr, "Received portal\n");
    if(s->topology_list == NULL){
        s->topology_list = (maplcm_topology_t *)calloc(1, sizeof(maplcm_topology_t));
        s->topology_list->portal_list.no_of_portals = 1;

        s->topology_list->portal_list.portals = (maplcm_portal_node_t *) calloc(1, sizeof(maplcm_portal_node_t));
        //not sure if this is the way to copy it
        memcpy(&s->topology_list->portal_list.portals[0], msg, sizeof(maplcm_portal_node_t));
        //s->tagged_places->places[0]
    }
    else{
        s->topology_list->portal_list.portals = (maplcm_portal_node_t *) realloc(s->topology_list->portal_list.portals, (s->topology_list->portal_list.no_of_portals + 1) * sizeof(maplcm_portal_node_t));
        memcpy(&s->topology_list->portal_list.portals[s->topology_list->portal_list.no_of_portals], msg, sizeof(maplcm_portal_node_t));
        s->topology_list->portal_list.no_of_portals++;

        for(int i=0; i < s->topology_list->portal_list.no_of_portals; i++){
            fprintf(stderr,"\t%d: %d\n", s->topology_list->portal_list.portals[i].type, s->topology_list->portal_list.portals[i].floor_no);
        }
    }

    if(s->topology_list)
        maplcm_topology_t_publish(s->lcm, "TOPOLOGY" , s->topology_list);
}


void topology_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                      const maplcm_topology_t *msg, void *user)
{
    state_t *s = (state_t *) user;

    fprintf(stderr,"Received Topology\n");

    if(!strcmp("MAP_SERVER_TOPOLOGY", channel)){
        fprintf(stderr,"Mapserver message - Ignoring\n");
        return;
    }

    //maybe we can write this to file here

    if(s->mode == 1 || s->mode == 2){
        if(0 && s->topology_list != NULL){
            maplcm_topology_t_destroy(s->topology_list);
            s->topology_list = NULL;
        }

        // Transform topology from local to global frame
        for (int i=0; i < msg->portal_list.no_of_portals; i++) {
            double xyz0_local[] = {msg->portal_list.portals[i].xy0[0],
                                  msg->portal_list.portals[i].xy0[1], 0};
            double xyz1_local[] = {msg->portal_list.portals[i].xy1[0],
                                  msg->portal_list.portals[i].xy1[1], 0};
            double xyz0_global[3], xyz1_global[3];

            bot_frames_transform_vec (s->frames, "local", "global", xyz0_local, xyz0_global);
            bot_frames_transform_vec (s->frames, "local", "global", xyz1_local, xyz1_global);

            msg->portal_list.portals[i].xy0[0] = xyz0_global[0];
            msg->portal_list.portals[i].xy0[1] = xyz0_global[1];
            msg->portal_list.portals[i].xy1[0] = xyz1_global[0];
            msg->portal_list.portals[i].xy1[1] = xyz1_global[1];
        }

        BotTrans local_to_global;
        bot_frames_get_trans (s->frames, "local", "global", &local_to_global);
        for (int i=0; i < msg->place_list.place_count; i++) {
            double xyz_local[] = {msg->place_list.trajectory[i].x, msg->place_list.trajectory[i].y, 0};
            double xyz_global[3];

            bot_frames_transform_vec (s->frames, "local", "global", xyz_local, xyz_global);

            double rpy_local[] = {0, 0, msg->place_list.trajectory[i].theta};
            double quat_local[4];
            double rpy_global[3], quat_global[3];

            bot_roll_pitch_yaw_to_quat (rpy_local, quat_local);
            bot_quat_mult (quat_global, local_to_global.rot_quat, quat_local);
            bot_quat_to_roll_pitch_yaw (quat_global, rpy_global);

            msg->place_list.trajectory[i].x = xyz_global[0];
            msg->place_list.trajectory[i].y = xyz_global[1];
            msg->place_list.trajectory[i].theta = rpy_global[2];
        }

        s->topology_list = maplcm_topology_t_copy(msg);
        fprintf(stderr,"Saving\n");


        //writing to file
        int channellen = strlen(channel);
        int64_t mem_sz = sizeof(lcm_eventlog_event_t) + channellen + 1 + rbuf->data_size;

        lcm_eventlog_event_t *le = (lcm_eventlog_event_t*) malloc(mem_sz);
        memset(le, 0, mem_sz);

        le->timestamp = rbuf->recv_utime;
        le->channellen = channellen;
        le->datalen = rbuf->data_size;

        le->channel = ((char*)le) + sizeof(lcm_eventlog_event_t);
        strcpy(le->channel, channel);
        le->data = le->channel + channellen + 1;

        assert((char*)le->data + rbuf->data_size == (char*)le + mem_sz);
        memcpy(le->data, rbuf->data, rbuf->data_size);

        if(0 != lcm_eventlog_write_event(s->write_log, le)) {
            fprintf(stderr, "Error\n");
        }
        else{
            fprintf(stderr,"Saved to file\n");
        }
    }
    else{
        fprintf(stderr,"Error : This should not be happening - Not in the correct mode\n");
    }
    //lets write to file
}

void sim_rect_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                      const obs_rect_list_t *msg, void *user)
{
    state_t *s = (state_t *) user;

    fprintf(stderr,"Received SIM rects\n");

    if(!strcmp("MAP_SERVER_RECTS", channel)){
        fprintf(stdout,"Mapserver message - Ignoring\n");
        return;
    }

    //maybe we can write this to file here
    if(s->mode == 1 || s->mode == 2){
        if(s->sim_rects != NULL){
            obs_rect_list_t_destroy(s->sim_rects);
        }

        s->sim_rects = obs_rect_list_t_copy(msg);
        fprintf(stderr,"Saving\n");

        // Transform simrects from local to global frame
        double xyz_local[] = {s->sim_rects->xy[0], s->sim_rects->xy[1], 0};
        double xyz_global[3];
        bot_frames_transform_vec (s->frames, "local", "global", xyz_local, xyz_global);
        s->sim_rects->xy[0] = xyz_global[0];
        s->sim_rects->xy[1] = xyz_global[1];

        for (int i=0; i<s->sim_rects->num_rects; i++) {
            // Update the dxy[2] and size[2]
            double dxyz_global[3], size_global[3], rpy_global[3], quat_global[4];
            double dxyz_local[3] = {s->sim_rects->rects[i].dxy[0], s->sim_rects->rects[i].dxy[1], 0};
            double size_local[3] = {s->sim_rects->rects[i].size[0], s->sim_rects->rects[i].size[1], 0};
            double rpy_local[3] = {0, 0, s->sim_rects->rects[i].theta};
            double quat_local[4];

            bot_frames_transform_vec (s->frames, "local", "global", dxyz_local, dxyz_global);
            bot_frames_transform_vec (s->frames, "local", "global", size_local, size_global);

            // Now for theta
            BotTrans local_to_global;
            bot_roll_pitch_yaw_to_quat (rpy_local, quat_local);
            bot_frames_get_trans (s->frames, "local", "global", &local_to_global);
            bot_quat_mult (quat_global, local_to_global.rot_quat, quat_local);
            bot_quat_to_roll_pitch_yaw (quat_global, rpy_global);

            s->sim_rects->rects[i].dxy[0] = dxyz_global[0];
            s->sim_rects->rects[i].dxy[1] = dxyz_global[1];

            s->sim_rects->rects[i].size[0] = size_global[0];
            s->sim_rects->rects[i].size[1] = size_global[1];

            s->sim_rects->rects[i].theta = rpy_global[2];
        }


        //writing to file
        int channellen = strlen(channel);
        int64_t mem_sz = sizeof(lcm_eventlog_event_t) + channellen + 1 + rbuf->data_size;

        lcm_eventlog_event_t *le = (lcm_eventlog_event_t*) malloc(mem_sz);
        memset(le, 0, mem_sz);

        le->timestamp = rbuf->recv_utime;
        le->channellen = channellen;
        le->datalen = rbuf->data_size;

        le->channel = ((char*)le) + sizeof(lcm_eventlog_event_t);
        strcpy(le->channel, channel);
        le->data = le->channel + channellen + 1;

        assert((char*)le->data + rbuf->data_size == (char*)le + mem_sz);
        memcpy(le->data, rbuf->data, rbuf->data_size);

        if(0 != lcm_eventlog_write_event(s->write_log, le)) {
            fprintf(stderr, "Error writing SIM_RECTS to file\n");
        }
        else{
            fprintf(stdout,"Saved SIM_RECTS to file\n");
        }
    }
    else{
        fprintf(stderr,"Error : This should not be happening - Not in the correct mode\n");
    }
    //lets write to file
}


void multi_gridmap_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                           const gmlcm_multi_gridmap_t *msg, void *user)
{
    state_t *s = (state_t *) user;

    if(!strcmp("MMAP_SERVER", channel)){
        fprintf(stderr,"Mapserver message - Ignoring\n");
        return;
    }

    //maybe we can write this to file here


    if(s->mode == 1 || s->mode == 2){
        if(s->multi_msg != NULL){
            gmlcm_multi_gridmap_t_destroy(s->multi_msg);
        }
        s->multi_msg = gmlcm_multi_gridmap_t_copy(msg);
        fprintf(stdout,"Received Multi-floor Map\n");

        //writing to file
        int channellen = strlen(channel);
        int64_t mem_sz = sizeof(lcm_eventlog_event_t) + channellen + 1 + rbuf->data_size;

        lcm_eventlog_event_t *le = (lcm_eventlog_event_t*) malloc(mem_sz);
        memset(le, 0, mem_sz);

        le->timestamp = rbuf->recv_utime;
        le->channellen = channellen;
        le->datalen = rbuf->data_size;

        le->channel = ((char*)le) + sizeof(lcm_eventlog_event_t);
        strcpy(le->channel, channel);
        le->data = le->channel + channellen + 1;

        assert((char*)le->data + rbuf->data_size == (char*)le + mem_sz);
        memcpy(le->data, rbuf->data, rbuf->data_size);

        if(0 != lcm_eventlog_write_event(s->write_log, le)) {
            fprintf(stderr, "Error writing multi-floor map\n");
        }
        else{
            fprintf(stdout, "Saved multi-floor map to file\n");
        }
    }
    else{
        fprintf(stderr,"Error : This should not be happening - Not in the correct mode\n");
    }
    //lets write to file
}

void lcm_map_request_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
			     const char *channel __attribute__((unused)),
			     const maplcm_map_request_msg_t *msg,
			     void *user)
{
    state_t *s = (state_t *) user;
    char* requester = msg->requesting_prog;
    fprintf(stderr,"Requested %s \n",requester);
    send_lcm_map(requester, s, s->current_floor_no);
}

void lcm_floor_map_request_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
                                   const char *channel __attribute__((unused)),
                                   const maplcm_map_request_msg_t *msg,
                                   void *user)
{
    state_t *s = (state_t *) user;
    char* requester = msg->requesting_prog;
    fprintf(stderr,"Floor Map Requested %s \n",requester);
    send_lcm_floor_map(requester, s->current_floor_no, s);
}

//request for a specific floor map
void lcm_floor_specific_map_request_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
                                            const char *channel __attribute__((unused)),
                                            const maplcm_map_request_msg_t *msg,
                                            void *user)
{
    state_t *s = (state_t *) user;
    char* requester = msg->requesting_prog;
    fprintf(stderr,"Specific Floor Map Requested %s - Floor : %d \n",requester, msg->floor_no);
    send_specific_lcm_floor_map(requester, msg->floor_no, s);
}

void lcm_mmap_request_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
			      const char *channel __attribute__((unused)),
			      const maplcm_map_request_msg_t *msg,
			      void *user)
{
    state_t *s = (state_t *) user;
    char* requester = msg->requesting_prog;
    fprintf(stderr,"Requested Multi Map %s \n",requester);
    if(s->multi_msg !=NULL){
        gmlcm_multi_gridmap_t_publish(s->lcm,"MMAP_SERVER", s->multi_msg);
    }
    else{
        fprintf(stderr,"No Multiple Map loaded - Unable to send");
    }
}

void floor_change_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)),
                          const maplcm_floor_change_msg_t * msg,
                          void * user  __attribute__((unused)))
{
    state_t *s = (state_t *) user;
    fprintf(stderr,"Floor Change Heard\n");

    if(s->multi_msg ==NULL){
        fprintf(stderr,"Error : No Map Loaded \n");
        return;
    }
    //we should validate the current floor
    //these are actual floor no's
    if(s->current_floor_no == msg->floor_no){
        fprintf(stderr,"No actual change in floor C:%d New: %d\n", s->current_floor_no,msg->floor_no );
        return;
    }
    else{
        if(get_floor_ind(s, msg->floor_no) >=0){
            fprintf(stderr,"Valid Floor Msg received\n");
            s->current_floor_no = msg->floor_no;

            int c_floor_ind = get_floor_ind(s, s->current_floor_no);

            s->multi_msg->current_floor_ind = c_floor_ind;

            send_lcm_map("Floor Change", s, s->current_floor_no);
            send_lcm_floor_map("Floor Change, current_floor_no",s->current_floor_no, s);
        }
        else{
            //do we still change ??? - might miss out on the floor change otherwise - if slam hasn't updated the map
            //the other option is to have slam provide us with the correct floor
            s->current_floor_no = msg->floor_no;
            fprintf(stderr,"No matching floor found\n");
            return;
        }
    }
}

int send_tagged_places(state_t *s){

    if(s->tagged_places !=NULL){
        maplcm_tagged_node_list_t_publish(s->lcm,"TAGGED_PLACE_LIST", s->tagged_places);
        return 0;
    }
    return 1;
}

int send_topology(state_t *s){
    if(s->topology_list != NULL){
        maplcm_topology_t_publish(s->lcm,"MAP_SERVER_TOPOLOGY", s->topology_list);
        return 0;
    }
    return 1;
}



int send_elevator_list(state_t *s){
    if(s->elevator_list !=NULL){
        maplcm_elevator_node_list_t_publish(s->lcm,"FINAL_ELEVATOR_LIST", s->elevator_list);
        return 0;
    }
    return 1;
}

int send_current_floor_status(state_t *s){
    maplcm_floor_status_msg_t msg;
    msg.utime = bot_timestamp_now();
    msg.floor_ind = get_floor_ind(s, s->current_floor_no);
    msg.floor_no = s->current_floor_no;
    maplcm_floor_status_msg_t_publish(s->lcm,"CURRENT_FLOOR_STATUS",&msg);

}

static gboolean
on_publish_timer (gpointer data)
{
    state_t *self = (state_t*) data;

    send_sim_rects_for_floor (self);
    send_topology (self);
    send_elevator_list (self);
    send_tagged_places (self);

    return TRUE;
}

//we need to add handling for this
void lcm_place_request_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
			       const char *channel __attribute__((unused)),
			       const maplcm_map_request_msg_t *msg,
			       void *user)
{
    state_t *s = (state_t *) user;
    fprintf(stderr,"Sending Places\n");
    if(send_tagged_places(s)!=0){
        fprintf(stderr, "No Place list in memory\n");
    }
    if(send_elevator_list(s)!=0){
        fprintf(stderr, "No Elevator list in memory\n");
    }
}

//we need to add handling for this
void lcm_topology_request_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
			       const char *channel __attribute__((unused)),
			       const maplcm_map_request_msg_t *msg,
			       void *user)
{
    state_t *s = (state_t *) user;
    fprintf(stderr,"Sending Topology\n");

    send_topology(s);
}

void lcm_tagged_places_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
                               const char *channel __attribute__((unused)),
                               const maplcm_tagged_node_list_t *msg,
                               void *user)
{
    fprintf(stderr,"Received Tagged places\n");

    state_t *s = (state_t *) user;

    if(s->tagged_places !=NULL){
        maplcm_tagged_node_list_t_destroy(s->tagged_places);
    }
    s->tagged_places = maplcm_tagged_node_list_t_copy(msg);

    int channellen = strlen(channel);
    int64_t mem_sz = sizeof(lcm_eventlog_event_t) + channellen + 1 + rbuf->data_size;

    lcm_eventlog_event_t *le = (lcm_eventlog_event_t*) malloc(mem_sz);
    memset(le, 0, mem_sz);

    le->timestamp = rbuf->recv_utime;
    le->channellen = channellen;
    le->datalen = rbuf->data_size;

    le->channel = ((char*)le) + sizeof(lcm_eventlog_event_t);
    strcpy(le->channel, channel);
    le->data = le->channel + channellen + 1;

    assert((char*)le->data + rbuf->data_size == (char*)le + mem_sz);
    memcpy(le->data, rbuf->data, rbuf->data_size);

    if(0 != lcm_eventlog_write_event(s->write_log, le)) {
        fprintf(stderr, "Error saving locations\n");
    }
}


void lcm_self_tagged_places_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
                                    const char *channel __attribute__((unused)),
                                    const maplcm_tagged_node_list_t *msg,
                                    void *user)
{
    fprintf(stderr,"Received Tagged places - self\n");

    state_t *s = (state_t *) user;


    //char *tagged_places_channel_name = "TAGGED_NODES";

    //int channellen = strlen(tagged_places_channel_name);
    int channellen = strlen(channel);
    int64_t mem_sz = sizeof(lcm_eventlog_event_t) + channellen + 1 + rbuf->data_size;

    lcm_eventlog_event_t *le = (lcm_eventlog_event_t*) malloc(mem_sz);
    memset(le, 0, mem_sz);

    le->timestamp = rbuf->recv_utime;
    le->channellen = channellen;
    le->datalen = rbuf->data_size;

    le->channel = ((char*)le) + sizeof(lcm_eventlog_event_t);
    //strcpy(le->channel, tagged_places_channel_name);
    strcpy(le->channel, channel);
    le->data = le->channel + channellen + 1;

    assert((char*)le->data + rbuf->data_size == (char*)le + mem_sz);
    memcpy(le->data, rbuf->data, rbuf->data_size);

    if(0 != lcm_eventlog_write_event(s->write_log, le)) {
        fprintf(stderr, "Error saving locations\n");
    }
}

void lcm_elevator_list_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
			       const char *channel __attribute__((unused)),
			       const maplcm_elevator_node_list_t *msg,
			       void *user)
{
    fprintf(stderr,"Elevator-list received\n");

    state_t *s = (state_t *) user;
    if(s->elevator_list !=NULL){
        maplcm_elevator_node_list_t_destroy(s->elevator_list);
    }
    s->elevator_list = maplcm_elevator_node_list_t_copy(msg);

    int channellen = strlen(channel);
    int64_t mem_sz = sizeof(lcm_eventlog_event_t) + channellen + 1 + rbuf->data_size;

    lcm_eventlog_event_t *le = (lcm_eventlog_event_t*) malloc(mem_sz);
    memset(le, 0, mem_sz);

    le->timestamp = rbuf->recv_utime;
    le->channellen = channellen;
    le->datalen = rbuf->data_size;

    le->channel = ((char*)le) + sizeof(lcm_eventlog_event_t);
    strcpy(le->channel, channel);
    le->data = le->channel + channellen + 1;

    assert((char*)le->data + rbuf->data_size == (char*)le + mem_sz);
    memcpy(le->data, rbuf->data, rbuf->data_size);

    if(0 != lcm_eventlog_write_event(s->write_log, le)) {
        fprintf(stderr, "Error saving Elevators\n");
    }
}

static void pose_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)),
                         const bot_core_pose_t * msg, void * user  __attribute__((unused)))
{

    state_t *s = (state_t *) user;


    if (s->latest_pose != NULL) {
        bot_core_pose_t_destroy(s->latest_pose);
    }
    s->latest_pose = bot_core_pose_t_copy(msg);
}

void speech_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)), const ripl_speech_cmd_t * msg,
		    void * user  __attribute__((unused)))
{
    state_t *s = (state_t *) user;
    char* cmd = msg->cmd_type;
    char* property = msg->cmd_property;
    if(strcmp(cmd,"MODE")==0){
        //mode has changed - we should be saving
        if(s->robot_mode == 0 && !strcmp(property,"navigation")){
            //we should make sure that we already have maps
            if(s->multi_msg == NULL){
                fprintf(stderr,"Error : No map and done with tour - Exiting\n");
                exit(-1);
            }
            fprintf(stderr,"Done with the tour\n");
            send_lcm_map("", s, s->current_floor_no);
            send_lcm_floor_map("", s->current_floor_no, s);
        }

        if(s->robot_mode == 0){
            fprintf(stderr,"Now in tourguide mode\n");
        }

        fprintf(stderr,"Mode Changed : %d \n",s->robot_mode);
    }
}

void floor_request_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
			   const char *channel __attribute__((unused)),
			   const maplcm_map_request_msg_t *msg,
			   void *user)
{
    state_t *s = (state_t *) user;
    char* requester = msg->requesting_prog;
    fprintf(stderr,"Floor Status Requested by %s \n",requester);
    send_current_floor_status(s);
}


void end_of_tour_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
			 const char * channel __attribute__((unused)),
			 const maplcm_tagged_node_t * msg,
			 void * user  __attribute__((unused)))
{
    state_t *s = (state_t *) user;
    char* cmd = msg->type;
    char* property = msg->label;

    if(strcmp(cmd,"mode")==0){
        //mode has changed - we should be saving
        if(s->robot_mode == 0 && !strcmp(property,"navigation")){
            //we should make sure that we already have maps
            if(s->multi_msg == NULL){
                fprintf(stderr,"Error : No map and done with tour - Exiting\n");
                exit(-1);
            }
            fprintf(stderr,"Done with the tour\n");
            send_lcm_map("", s, s->current_floor_no);
            send_lcm_floor_map("", s->current_floor_no, s);
        }

        if(s->robot_mode == 0){
            fprintf(stderr,"Now in tourguide mode\n");
        }

        fprintf(stderr,"Mode Changed : %d \n",s->robot_mode);
    }

}


void subscribe_messages(state_t *s){
    //map requests
    maplcm_map_request_msg_t_subscribe(s->lcm,"MAP_REQUEST",lcm_map_request_handler, s);

    maplcm_map_request_msg_t_subscribe(s->lcm,"FMAP_REQUEST",lcm_floor_map_request_handler, s);

    //request a specific floor map
    maplcm_map_request_msg_t_subscribe(s->lcm,"SFMAP_REQUEST",lcm_floor_specific_map_request_handler, s);

    maplcm_map_request_msg_t_subscribe(s->lcm,"MMAP_REQUEST",lcm_mmap_request_handler, s);

    maplcm_map_request_msg_t_subscribe(s->lcm,"PLACE_REQUEST",lcm_place_request_handler, s);

    maplcm_map_request_msg_t_subscribe(s->lcm,"TOPOLOGY_REQUEST",lcm_topology_request_handler, s);



    //subscription for the tagged locations

    //-Not handled-yet
    maplcm_tagged_node_list_t_subscribe(s->lcm, "TAGGED_NODES", lcm_tagged_places_handler, s);

    //-Not handled-yet
    maplcm_elevator_node_list_t_subscribe(s->lcm, "ELEVATOR_LIST", lcm_elevator_list_handler, s);


    bot_core_pose_t_subscribe(s->lcm, "POSE", pose_handler, s);


    //add the other handlers - these should include the new portal messages and the place node messages

    if(s->mode == 1 || s->mode == 2){   //maps being published by isam slam
        gmlcm_gridmap_t_subscribe(s->lcm, "FINAL_SLAM", gridmap_handler, s);
        //if we are to save the latest - then we should also subscribe to the full stream
        if(s->latest){
            gmlcm_multi_gridmap_t_subscribe(s->lcm, "MULTI_FLOOR_MAPS", multi_gridmap_handler, s);
        }
        //subsribes to the final slam maps only
        gmlcm_multi_gridmap_t_subscribe(s->lcm, "FINAL_MULTI_SLAM", multi_gridmap_handler, s);
    }

    //Might be deprected - delete if not used
    ripl_speech_cmd_t_subscribe(s->lcm, "TAGGING_CHANNEL", speech_handler, s);

    //should use a different message for this
    maplcm_tagged_node_t_subscribe(s->lcm, "WHEELCHAIR_MODE", end_of_tour_handler, s);

    if(s->mode == 1 || s->mode == 2){
        obs_rect_list_t_subscribe(s->lcm,"SIM_RECTS", sim_rect_handler, s);

        maplcm_topology_t_subscribe(s->lcm,"TOPOLOGY", topology_handler, s);

        //maplcm_tagged_node_t_subscribe(s->lcm,"ADD_TAGGED_PLACE", place_update_handler, s);
        maplcm_place_node_t_subscribe(s->lcm,"ADD_PLACE_NODE", place_update_handler, s);

        maplcm_portal_node_t_subscribe(s->lcm,"ADD_PORTAL_NODE", portal_update_handler, s);

    }

    maplcm_floor_change_msg_t_subscribe(s->lcm, "FLOOR_CHANGE", floor_change_handler, s);
    maplcm_map_request_msg_t_subscribe(s->lcm, "FLOOR_STATUS_REQUEST", floor_request_handler, s); //send both the current floor and the floor map
}

static void usage(const char *name)
{
    fprintf(stderr, "usage: %s [options]\n"
            "\n"
            "  -v, --verbose                     Verbose output \n"
            "  -m, --mode                        Mode (listen/publish/append)            \n"
            "  -p, --path                        Path to load/save file (otherwise will load from default map location    \n"
            "  -n, --name                        Name of Map file    \n"
            "  -a, --append_filename             Name of Appended Map file    \n"
            "  -f, --floor                       Floor number \n"
            "  -l, --latest                      Save Latest Map \n", name);
}

int main(int argc, char *argv[])
{
    GMainLoop *_mainloop;
    // initialize GLib threading
    g_thread_init(NULL);

    //simple server that reads from a log file and publishes the map
    state_t *s = calloc(1, sizeof(state_t));

    s->current_floor_no = -1;
    s->latest_pose = NULL;
    s->tagged_places = NULL;
    s->verbose = 0;
    char c;
    const char *optstring = "p:m:a:lf:n:v";

    struct option long_opts[] = {{ "mode", required_argument, 0, 'm'}, //mode can be publish or listen
                                 { "latest", no_argument, 0, 'l'}, //save the latest slam map
                                 { "path", required_argument, 0, 'p' }, //path to the file - overrides the default path
                                 { "name", required_argument, 0, 'n' }, //file name
                                 { "append_filename", required_argument, 0, 'a' }, //file name
                                 { "floor", required_argument, 0, 'f' }, //current floor
                                 { "verbose", no_argument, 0, 'v' },
                                 { 0, 0, 0, 0 } };

    const char * file_path = NULL;
    char * file_name = NULL;
    char * outfile_name = NULL;
    char *mode = NULL;
    while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
        switch (c) {
        case 'p':
            file_path =  strdup(optarg);
            break;
        case 'n':
            file_name =  strdup(optarg);
            break;
        case 'a':
            outfile_name =  strdup(optarg);
            break;
        case 'l':
            s->latest = 1;
            break;
        case 'f':
            s->current_floor_no = atoi(optarg); //actual no of the floor
            fprintf(stderr,"Current Floor : %d\n", s->current_floor_no);
            break;
        case 'm':
            mode =  strdup(optarg);
            if(!strcmp(mode,"publish") || !strcmp(mode,"Publish")){
                s->mode = 0;
            }
            else if(!strcmp(mode,"listen") || !strcmp(mode,"Listen")){
                s->mode = 1;
            }
            else if(!strcmp(mode,"append") || !strcmp(mode,"append")){
                s->mode = 2;
            }
            free(mode);
            break;
        case 'v':
            s->verbose = 1;
            break;
        default:
            usage(argv[0]);
            return 1;
        }
    }

    // Doesn't make sense to specify append file when not in append mode
    if (s->mode != 2 && outfile_name)
        fprintf (stderr, "Append file not used when not in append mode\n");

    //const char * default_path = NULL;

    char full_path[1024];

    if(file_path == NULL){
        //get default path
        file_path = getMapsPath();
        fprintf(stderr,"Getting Map path\n");

    }
    else{
        fprintf(stdout,"Using custom path\n");
    }

    sprintf(full_path,"%s/%s", file_path, file_name);

    char outfull_path[1024];

    sprintf(outfull_path,"%s/%s", file_path, outfile_name);

    //combine path

    fprintf(stdout, " Map Path : %s\n", full_path);

    //gmlcm_multi_gridmap_t *multi_msg = NULL;
    //gmlcm_gridmap_t *to_pub = NULL;

    s->lcm = bot_lcm_get_global(NULL);
    s->multi_msg = NULL;
    s->single_map = NULL;

    BotParam * param = bot_param_new_from_server (s->lcm, 0);
    s->frames = bot_frames_get_global (s->lcm, param);

    //hmm - this is read-only
    fprintf(stdout, "Mode : %d\n", s->mode);

    if(s->mode == 0 || s->mode == 2){
        s->read_log = lcm_eventlog_create(full_path, "r");

        s->single_map = calloc(1, sizeof(gmlcm_gridmap_t));
        s->multi_msg = calloc(1,sizeof(gmlcm_multi_gridmap_t));

        if (!s->read_log) {
            fprintf (stderr, "Unable to open source logfile for reading %s\n", full_path);
            return 1;
        }

        if(s->mode == 2){
            fprintf (stdout, "In append Mode\n");
            //prevents us from overwriting files
            if(g_file_test(outfull_path, G_FILE_TEST_EXISTS)){
                fprintf(stderr,"Append Mode: File (%s) already exists\n", outfull_path);
                return 1;
            }
            s->write_log = lcm_eventlog_create(outfull_path, "w");

            if (s->write_log == NULL) {
                fprintf (stderr, "Unable to open source logfile (%s) for writing`w\n", outfull_path);
                return 1;
            }
            else{
                fprintf(stdout, "Opened file (%s) to write\n", outfull_path);
            }
        }
    }

    else if(s->mode == 1){
        fprintf(stderr, "In create Mode\n");
        //prevents us from overwriting files
        if(g_file_test(full_path, G_FILE_TEST_EXISTS)){
            fprintf(stderr,"Listen Mode: File (%s) already exists\n", full_path);
            return 1;
        }
        s->write_log = lcm_eventlog_create(full_path, "w");
        fprintf(stdout,"Opening file (%s) for writing\n", full_path);

        if (s->write_log == NULL) {
            fprintf (stderr, "Unable to open source logfile (%s) for writing\n", full_path);
            return 1;
        }
        else{
            fprintf(stdout, "Opened file to write\n");
        }
    }
    else{
        fprintf(stderr,"Incorrect mode\n");
        return 1;
    }

    /*if(file_path !=NULL)
      free(file_path);*/
    if(file_name !=NULL)
        free(file_name);

    lcm_eventlog_event_t *slam_event = NULL;
    lcm_eventlog_event_t *final_slam_event = NULL;
    lcm_eventlog_event_t *elevator_event = NULL;
    lcm_eventlog_event_t *tagging_event = NULL;
    lcm_eventlog_event_t *topology_event = NULL;
    lcm_eventlog_event_t *simrects_event = NULL;

    if(s->mode == 0 || s->mode == 2){
        char *basic_slam_channel_name = "MULTI_FLOOR_MAPS";
        char *final_slam_channel_name = "FINAL_MULTI_SLAM"; //this should override basic slam channel messages
        int decoded = 0;
        int final_slam_found = 0; //we found a final slam message in the logs
        int tagged_places_found = 0;
        int topology_found = 0;
        int elevator_list_found = 0;
        int sim_rects_found = 0;

        char *tagged_places_channel_name = "TAGGED_NODES";
        char *elevator_list_channel_name = "ELEVATOR_LIST";
        char *topology_channel_name = "TOPOLOGY";
        char *sim_rects_channel_name = "SIM_RECTS";

        for (lcm_eventlog_event_t *event = lcm_eventlog_read_next_event (s->read_log);
             event != NULL;
             event = lcm_eventlog_read_next_event (s->read_log)) {

            if (s->verbose)
                fprintf(stdout,"Channel : %s\n",event->channel);

            int decode_status;
            if (strcmp (basic_slam_channel_name, event->channel) == 0) {
                if(slam_event != NULL){
                    lcm_eventlog_free_event (slam_event);
                }
                slam_event = event;
                //memset (s->multi_msg), 0, sizeof (gmlcm_multi_gridmap_t));
                decode_status =  gmlcm_multi_gridmap_t_decode (event->data, 0, event->datalen, s->multi_msg);
                if (decode_status < 0)
                    fprintf (stderr, "Error %d decoding message\n", decode_status);
                else
                    decoded = 1;
            }

            else if (strcmp (sim_rects_channel_name, event->channel) == 0) {
                if(simrects_event != NULL){
                    lcm_eventlog_free_event (simrects_event);
                }
                simrects_event = event;

                if(s->sim_rects !=NULL){
                    obs_rect_list_t_destroy(s->sim_rects);
                }
                s->sim_rects = calloc(1,sizeof(obs_rect_list_t));

                //memset (s->multi_msg), 0, sizeof (gmlcm_multi_gridmap_t));
                decode_status = obs_rect_list_t_decode (event->data, 0, event->datalen, s->sim_rects);
                if (decode_status < 0)
                    fprintf (stderr, "Error %d decoding message\n", decode_status);
                else{
                    sim_rects_found = 1;
                }
            }

            else if (strcmp (final_slam_channel_name, event->channel) == 0) {
                if(final_slam_event != NULL){
                    lcm_eventlog_free_event (final_slam_event);
                }
                final_slam_event = event;

                //memset (s->multi_msg), 0, sizeof (gmlcm_multi_gridmap_t));
                decode_status =  gmlcm_multi_gridmap_t_decode (event->data, 0, event->datalen, s->multi_msg);
                if (decode_status < 0)
                    fprintf (stderr, "Error %d decoding message\n", decode_status);
                else{
                    decoded = 1;
                    final_slam_found = 1;
                    //lcm_eventlog_free_event (event);
                }
            }

            else if (strcmp (tagged_places_channel_name, event->channel) == 0) {
                fprintf(stdout,"Places found \n");
                if(tagging_event != NULL){
                    lcm_eventlog_free_event (tagging_event);
                }
                tagging_event = event;

                if(s->tagged_places !=NULL){
                    maplcm_tagged_node_list_t_destroy(s->tagged_places);
                }
                s->tagged_places = calloc(1,sizeof(maplcm_tagged_node_list_t));

                //memset (s->multi_msg), 0, sizeof (gmlcm_multi_gridmap_t));
                decode_status =   maplcm_tagged_node_list_t_decode (event->data, 0, event->datalen, s->tagged_places);
                if (decode_status < 0)
                    fprintf (stderr, "Error %d decoding message\n", decode_status);
                else{
                    tagged_places_found = 1;
                }
            }
            else if (strcmp (topology_channel_name, event->channel) == 0) {
                fprintf(stdout,"Topology found \n");
                if(topology_event != NULL){
                    lcm_eventlog_free_event (topology_event);
                }
                topology_event = event;

                if(s->topology_list !=NULL){
                    maplcm_topology_t_destroy(s->topology_list);
                    s->topology_list = NULL;
                }

                s->topology_list = calloc(1,sizeof(maplcm_topology_t));

                //memset (s->multi_msg), 0, sizeof (gmlcm_multi_gridmap_t));
                decode_status =   maplcm_topology_t_decode (event->data, 0, event->datalen, s->topology_list);
                if (decode_status < 0)
                    fprintf (stderr, "Error %d decoding message\n", decode_status);
                else{
                    topology_found = 1;
                }
            }

            else if (strcmp (elevator_list_channel_name, event->channel) == 0) {

                if(elevator_event != NULL){
                    lcm_eventlog_free_event (elevator_event);
                }
                elevator_event = event;

                if(s->elevator_list !=NULL){
                    maplcm_elevator_node_list_t_destroy(s->elevator_list);
                }
                s->elevator_list = calloc(1,sizeof(maplcm_elevator_node_list_t));

                //memset (s->multi_msg), 0, sizeof (gmlcm_multi_gridmap_t));
                decode_status =   maplcm_elevator_node_list_t_decode (event->data, 0, event->datalen, s->elevator_list);
                if (decode_status < 0)
                    fprintf (stderr, "Error %d decoding message\n", decode_status);
                else{
                    elevator_list_found = 1;
                }
            }
            else {
                lcm_eventlog_free_event (event);
            }
        }

        if(!decoded){
            fprintf(stderr,"Error : Could not find message\n");
            //return 0;
        }
        else {
            if(s->mode == 2){
                fprintf(stdout, "Writing to appended log\n");
                //write the log events back to log
                if(slam_event){
                    lcm_eventlog_write_event(s->write_log, slam_event);
                }
                if(final_slam_event){
                    lcm_eventlog_write_event(s->write_log, final_slam_event);
                }
                if(elevator_event){
                    lcm_eventlog_write_event(s->write_log, elevator_event);
                }
                if(tagging_event){
                    lcm_eventlog_write_event(s->write_log, tagging_event);
                }
                if(topology_event){
                    lcm_eventlog_write_event(s->write_log, topology_event);
                }
                if(simrects_event){
                    lcm_eventlog_write_event(s->write_log, simrects_event);
                }
            }

            if(final_slam_found){
                fprintf(stdout,"Found final slam message \n");
            }
            //we should validate the current floor
            int c_floor_ind = get_floor_ind(s, s->current_floor_no);

            s->multi_msg->current_floor_ind = c_floor_ind;

            if(c_floor_ind == -1){
                fprintf(stderr,"Invalid Current Floor Given\n");
                s->current_floor_no = -1;
            }

            fprintf(stdout, "Loaded Map - Publishing Latest \n");
            gmlcm_multi_gridmap_t_publish(s->lcm,"MMAP_SERVER", s->multi_msg);
            send_lcm_map("Floor Change", s, s->current_floor_no);
            send_lcm_floor_map("Floor Change, current_floor_no",s->current_floor_no, s);
            send_current_floor_status(s);
        }
        if(tagged_places_found){
            fprintf(stdout, "Loaded Tagged Places - Publishing Latest \n");
            send_tagged_places(s);
        }
        else{
            fprintf(stdout, "No tagged places loaded\n");

        }
        if(elevator_list_found){
            send_elevator_list(s);
        }
        else{
            fprintf(stdout, "No elevator list loaded\n");
        }
        if(topology_found){
            fprintf(stdout, "Topology list loaded\n");
            send_topology(s);
        }
        else{
            fprintf(stdout, "No topology list loaded\n");
        }
        if(sim_rects_found){
            fprintf(stdout, "Sim rects loaded\n");
            send_sim_rects_for_floor(s);
        }
        else{
            fprintf(stdout, "No sim rects loaded\n");
        }


        if(slam_event){
            lcm_eventlog_free_event (slam_event);
        }
        if(final_slam_event){
            lcm_eventlog_free_event (final_slam_event);
        }
        if(simrects_event){
            lcm_eventlog_free_event (simrects_event);
        }
        if(elevator_event){
            lcm_eventlog_free_event (elevator_event);
        }
        if(tagging_event){
            lcm_eventlog_free_event (tagging_event);
        }
        if(topology_event){
            lcm_eventlog_free_event(topology_event);
        }
    }

    // If we are in publish mode, setup a timer to periodically republish info
    if(s->mode == 0)
        s->publish_timer_id = g_timeout_add (1000/PUBLISH_RATE, on_publish_timer, s);

    subscribe_messages(s);

    _mainloop = g_main_loop_new (NULL, FALSE);
    //signal_pipe_glib_quit_on_kill ();
    //glib_mainloop_attach_lcm (s->lcm);
    bot_glib_mainloop_attach_lcm (s->lcm);
    bot_signal_pipe_glib_quit_on_kill (_mainloop);
    // main loop
    g_main_loop_run (_mainloop);

    //handle message requests
    /*while (1)
      lcm_handle (s->lcm);
    */

    bot_glib_mainloop_detach_lcm (s->lcm);
    lcm_destroy (s->lcm);
    if(s->read_log){
        lcm_eventlog_destroy (s->read_log);
    }
    if(s->write_log){
        lcm_eventlog_destroy (s->write_log);
    }
    free(s->multi_msg);
    free(s->single_map);

    //gmlcm_multi_gridmap_t_decode_cleanup (&(s->multi_msg));
    //now should have the lcm handling here

    return 0;
}
