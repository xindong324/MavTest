#include"define.h"
#include"mavlink_avoid_errors.h"
#include"MAVLINK/common/mavlink.h"

String FileName;

mavlink_heartbeat_t  heartbeat;
mavlink_sys_status_t sys;
mavlink_attitude_t   attitude;
mavlink_gps_raw_int_t gpsPos;
mavlink_global_position_int_t position;
mavlink_command_long_t command;
mavlink_command_ack_t ack;
mavlink_system_time_t sys_time;
mavlink_system_t mavlink_system;
int lastTime=0;
static int8_t control_mode = STABILIZE;
unsigned int displayInt;
int k;    // counter variable
bool is=false;
//const int delayTime;
