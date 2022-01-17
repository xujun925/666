#include <termios.h>

#include <drivers/drv_hrt.h>

#include <systemlib/mavlink_log.h>

//#include <px4_posix.h>

//#include <vector>

//#include <deque>
#include "drivers/drv_pwm_output.h"
#include <uORB/topics/offboard_control_mode.h>

#include <uORB/topics/position_setpoint_triplet.h>
#include <unistd.h>
#include <stdio.h>
#include <uORB/topics/home_position.h>
#include <string.h>
#include <stdlib.h>
//#include <px4_config.h>
//#include <px4_tasks.h>
//#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_log.h>

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_command_ack.h>
//#include <geo/geo.h>
#include <poll.h>

#include <uORB/topics/rc_channels.h>
//#include <uORB/topics/output_pwm.h>
//#include <nuttx/fs/ioctl.h>
#include "drivers/drv_pwm_output.h"
#include "systemlib/err.h"
#include <sys/ioctl.h>
//#include "systemlib/systemlib.h"
//#include "systemlib/param/param.h"
#include <sys/stat.h>
#include <fcntl.h>
//#include <arch/board/board.h>
//#include <drivers/drv_gpio.h>
#include <sys/types.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <drivers/drv_input_capture.h>

//#include <px4_config.h>
//#include <px4_tasks.h>
//#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

// system libraries
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>

// internal libraries
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>

// Include uORB and the required topics for this app
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>                // this topics hold the acceleration data
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/vehicle_attitude.h>                  // this topic holds the orientation of the hippocampus
#include <uORB/topics/safety.h>
//#include <uORB/topics/a02.h>
//#include <uORB/topics/a03pid_out.h>
//#include<uORB/topics/a04sp_GPS.h>
//#include<uORB/topics/a04sp_GPS2.h>
//#include<uORB/topics/a04sp_GPS3.h>

#include "commander/commander_helper.h"
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_pwm_output.h>
//#include "mc_pos_control/PositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
//#include "mc_pos_control/Utility/ControlMath.hpp"
#include <px4_defines.h>

//#include <px4_config.h>
#include <px4_defines.h>
//#include <px4_module_params.h>
//#include <px4_tasks.h>
//#include <px4_module.h>
//#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <lib/hysteresis/hysteresis.h>
#include <commander/px4_custom_mode.h>

#include <uORB/Publication.hpp>
//#include <uORB/PublicationQueued.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/landing_gear.h>
#include <float.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>
#include <controllib/blocks.hpp>
#include <uORB/topics/distance_sensor.h>


//#include <lib/FlightTasks/FlightTasks.hpp>
//#include <lib/WeatherVane/WeatherVane.hpp>
#include "mc_pos_control/Takeoff/Takeoff.hpp"


#define PX4FMU_DEVICE_PATH	"/dev/px4fmu"




extern "C" __EXPORT int a_main(int argc, char *argv[]);//主函数
int a_thread(int argc, char *argv[]);//编队线程

static bool thread_running = false;//线程运行标志
static bool thread_should_exit = false;//线程结束标志
static orb_advert_t mavlink_log_pub = 0;
static int daemon_task;


class main_task
{
public:
    void run();

private:
    int a[10]={4,8,12,12,8,4,14,16,18,20};
    int b[10]={-4,-8,-4,4,8,4,0,0,0,0};
    vehicle_global_position_s _vehicle_global_position;
        distance_sensor_s _distance_sensor;
    float x=0,y=0;
    double lat,lon;
    map_projection_reference_s _ref_pos{};
    uint64_t time_tick=hrt_absolute_time();
    int flag=0;
    vehicle_local_position_s _vehicle_local_position;
    vehicle_status_s _status;
    vehicle_command_s _command = {};
    offboard_control_mode_s ocm{};
    position_setpoint_triplet_s _pos_sp_triplet{};
    vehicle_command_ack_s _ack{};
    int vehicle_command_ack_sub = orb_subscribe(ORB_ID(vehicle_command_ack));//订阅vehicle_command_ack


    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
    uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};


    uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
    uORB::Publication<vehicle_command_s>		_vehicle_command_pub{ORB_ID(vehicle_command)};
    uORB::Publication<position_setpoint_triplet_s>		_position_setpoint_triplet_pub{ORB_ID(position_setpoint_triplet)};
    uORB::Publication<vehicle_local_position_s>		_vehicle_local_position_pub{ORB_ID(vehicle_local_position)};

    px4_pollfd_struct_t fds;



};


void main_task::run()
{

    _vehicle_status_sub.copy(&_status);
    PX4_INFO("system_id:%d\t component_id:%d\r\n",  _status.system_id,_status.component_id);

    _command.target_system = _status.system_id;//system_id写入
    _command.target_component = _status.component_id;//component_id写入

    memset(&ocm, 0, sizeof(offboard_control_mode_s));
    ocm.ignore_acceleration_force = true;//必须要强制关闭加速度控制才能进行速度控制,位置控制
    memset(&_pos_sp_triplet, 0, sizeof(position_setpoint_triplet_s));//全部清0
    _pos_sp_triplet.timestamp = hrt_absolute_time();//写当前时间，非必须

    _vehicle_local_position_sub.copy(&_vehicle_local_position);
//    _vehicle_global_position_sub.copy(&_vehicle_global_position);
     map_projection_init(&_ref_pos, _vehicle_local_position.ref_lat, _vehicle_local_position.ref_lon);
//     map_projection_project(&_ref_pos,
//                            _vehicle_global_position.lat,_vehicle_global_position.lon,
//                            &x,
//                            &y);
 map_projection_reproject(&_ref_pos,10,10,&lat,&lon);

     PX4_INFO("lat=%3.7f",lat);
     PX4_INFO("lon=%3.7f",lon);


    while(1)//此循环进入offboard

    {
            _vehicle_status_sub.copy(&_status);
if((_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD)&&(_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED))
    break;
        ocm.timestamp = hrt_absolute_time(); //赋予系统时间戳
        _offboard_control_mode_pub.publish(ocm);


        _command.command = vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE;
        _command.param1 = 1.0f;
        _vehicle_command_pub.publish(_command);
        usleep(10000);

        _command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        _vehicle_command_pub.publish(_command);
        usleep(10000);
    }


    _vehicle_local_position_sub.copy(&_vehicle_local_position);
    PX4_INFO("x=%f",(double)_vehicle_local_position.x);
    PX4_INFO("y=%f",(double)_vehicle_local_position.y);

    while(1)//此循环为业务逻辑
    {
        if(flag==0)//升至5米
        {
            flag++;
            memset(&_pos_sp_triplet, 0, sizeof(position_setpoint_triplet_s));//全部清0
            _pos_sp_triplet.current.valid = true;
            _pos_sp_triplet.current.position_valid = true;
            _pos_sp_triplet.current.x=_vehicle_local_position.x;
            _pos_sp_triplet.current.y =_vehicle_local_position.y;
            _pos_sp_triplet.current.z=_vehicle_local_position.z-5;
            _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
            _pos_sp_triplet.current.alt_valid = true;
            _position_setpoint_triplet_pub.publish(_pos_sp_triplet);
             time_tick=hrt_absolute_time();
             mavlink_log_critical(&mavlink_log_pub, "takeoff");
        }

        if(flag==1&&(hrt_absolute_time()-time_tick>10000000))
        {
            flag++;
            memset(&_pos_sp_triplet, 0, sizeof(position_setpoint_triplet_s));//全部清0
            _pos_sp_triplet.current.valid = true;
            _pos_sp_triplet.current.position_valid = true;
            _pos_sp_triplet.current.x=_vehicle_local_position.x+10;
            _pos_sp_triplet.current.y =_vehicle_local_position.y+10;
            _pos_sp_triplet.current.z=-5;
            _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
            _pos_sp_triplet.current.alt_valid = true;
            _position_setpoint_triplet_pub.publish(_pos_sp_triplet);
             time_tick=hrt_absolute_time();
             mavlink_log_critical(&mavlink_log_pub, "position control");
        }
        if(flag==2&&(hrt_absolute_time()-time_tick>10000000))
        {
            flag++;
            memset(&_pos_sp_triplet, 0, sizeof(position_setpoint_triplet_s));//全部清0
            _pos_sp_triplet.current.valid = true;
            _pos_sp_triplet.current.position_valid = true;
            _pos_sp_triplet.current.yaw_valid = true;
            _pos_sp_triplet.current.yaw=1;
            _pos_sp_triplet.current.x=_vehicle_local_position.x+10;
            _pos_sp_triplet.current.y =_vehicle_local_position.y+10;
            _pos_sp_triplet.current.z=-5;
            _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
            _pos_sp_triplet.current.alt_valid = true;
            _position_setpoint_triplet_pub.publish(_pos_sp_triplet);
             time_tick=hrt_absolute_time();
             mavlink_log_critical(&mavlink_log_pub, "yaw control");
        }
        if(flag==3&&(hrt_absolute_time()-time_tick>10000000))
        {
            flag++;
            memset(&_pos_sp_triplet, 0, sizeof(position_setpoint_triplet_s));//全部清0
            _pos_sp_triplet.current.valid = true;
            _pos_sp_triplet.current.position_valid = true;
            _pos_sp_triplet.current.yawspeed_valid = true;
            _pos_sp_triplet.current.yawspeed=-1;
            _pos_sp_triplet.current.x=_vehicle_local_position.x+10;
            _pos_sp_triplet.current.y =_vehicle_local_position.y+10;
            _pos_sp_triplet.current.z=-5;
            _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
            _pos_sp_triplet.current.alt_valid = true;
            _position_setpoint_triplet_pub.publish(_pos_sp_triplet);
             time_tick=hrt_absolute_time();
             mavlink_log_critical(&mavlink_log_pub, "yaw speed control");
        }
        if(flag==4&&(hrt_absolute_time()-time_tick>10000000))
        {
            flag++;
            memset(&_pos_sp_triplet, 0, sizeof(position_setpoint_triplet_s));//全部清0
            _pos_sp_triplet.current.valid = true;
            _pos_sp_triplet.current.velocity_valid = true;
            _pos_sp_triplet.current.vx=-1;
            _pos_sp_triplet.current.vy =-1;
            _pos_sp_triplet.current.z=-5;
            _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_VELOCITY;
            _pos_sp_triplet.current.velocity_frame = position_setpoint_s::VELOCITY_FRAME_LOCAL_NED;
            _pos_sp_triplet.current.alt_valid = true;
            _position_setpoint_triplet_pub.publish(_pos_sp_triplet);
             time_tick=hrt_absolute_time();
             mavlink_log_critical(&mavlink_log_pub, "velocity control");
        }
        if(flag==5&&(hrt_absolute_time()-time_tick>10000000))
        {
            flag++;
            memset(&_pos_sp_triplet, 0, sizeof(position_setpoint_triplet_s));//全部清0
            _pos_sp_triplet.current.valid = true;
            _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
            _position_setpoint_triplet_pub.publish(_pos_sp_triplet);
             time_tick=hrt_absolute_time();
             mavlink_log_critical(&mavlink_log_pub, "land");
        }

       ocm.timestamp = hrt_absolute_time();   //以10HZ的频率发布OFFBOARD心跳包

     _offboard_control_mode_pub.publish(ocm);

        usleep(100000);



        // return 0;
    }
}


int a_main(int argc, char *argv[])
{
    if (argc < 2) {
        mavlink_log_critical(&mavlink_log_pub, "[biandui]mission command");
    }
    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            mavlink_log_critical(&mavlink_log_pub, "[biandui]already running");
            //    exit(0);
        }
        thread_should_exit = false;
        thread_running = true;
        daemon_task = px4_task_spawn_cmd("a",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_MAX - 5,
                                         3000,
                                         a_thread,
                                         &argv[2]);
        return 0;
        //      exit(0);
    }
    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        //    exit(0);
    }
    mavlink_log_critical(&mavlink_log_pub, "unrecognized command");
    //    exit(1);
    return 0;
}




int a_thread(int argc, char *argv[])
{
    main_task a;
    a.run();
    return 0;
}
