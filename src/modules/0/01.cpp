#include <termios.h>

#include <drivers/drv_hrt.h>

#include <systemlib/mavlink_log.h>

#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include "drivers/drv_pwm_output.h"
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <unistd.h>
#include <stdio.h>
#include <uORB/topics/home_position.h>
#include <string.h>
#include <stdlib.h>
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

#include <float.h>
#include <mathlib/mathlib.h>

#include <px4_defines.h>

#include <px4_defines.h>
//#include <px4_module_params.h>

//#include <px4_module.h>

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
//#include <lib/FlightTasks/FlightTasks.hpp>

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>

#define PX4FMU_DEVICE_PATH	"/dev/px4fmu"




static orb_advert_t mavlink_log_pub = 0;

static bool thread_running = false;

static bool thread_should_exit = false;

static int daemon_task;
hrt_abstime _ref_timestamp = 0;
struct map_projection_reference_s _ref_pos;
float _ref_alt = 0.0f;
// int manual_control_setpoint_sub;

int uart_net_thread(int argc, char *argv[]);










static int uart_init(char * uart_name);
 static int set_uart_baudrate(const int fd, unsigned int baud);
 int set_uart_baudrate(const int fd, unsigned int baud)
{
 int speed;
switch (baud)
 {
case 9600: speed = B9600; break;
 case 19200: speed = B19200; break;
case 38400: speed = B38400; break;
case 57600: speed = B57600; break;
 case 115200: speed = B115200; break;
 default: warnx("ERR: baudrate: %d\n", baud);
return -EINVAL;
 }
struct termios uart_config;
 int termios_state;
 tcgetattr(fd, &uart_config);
 uart_config.c_oflag &= ~ONLCR;

 uart_config.c_cflag &= ~(CSTOPB | PARENB);

 if ((termios_state = cfsetispeed(&uart_config, speed)) < 0)
 {
 warnx("ERR: %d (cfsetispeed)\n", termios_state);
return false; }
 if ((termios_state = cfsetospeed(&uart_config, speed)) < 0)
 {
 warnx("ERR: %d (cfsetospeed)\n", termios_state);
 return false;
 }
if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0)
{
warnx("ERR: %d (tcsetattr)\n", termios_state);
 return false;
 }
 return true;
 }
int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
 if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
  return serial_fd;
}









extern "C" __EXPORT int uart_net_main(int argc, char *argv[]);


int uart_net_main(int argc, char *argv[])

{

    if (argc < 2) {

        mavlink_log_critical(&mavlink_log_pub, "[uart_net]mission command");

    }


    if (!strcmp(argv[1], "start")) {

        if (thread_running) {

            mavlink_log_critical(&mavlink_log_pub, "[uart_net]already running");

            exit(0);

        }



        thread_should_exit = false;

        thread_running = true;

        daemon_task = px4_task_spawn_cmd("uart_net",

                                         SCHED_DEFAULT,

                                         SCHED_PRIORITY_MAX - 5,

                                         3000,

                                         uart_net_thread,

                                         &argv[2]);


        return 0;

        exit(0);

    }



    if (!strcmp(argv[1], "stop")) {

        thread_should_exit = true;

        exit(0);

    }



    mavlink_log_critical(&mavlink_log_pub, "unrecognized command");

    exit(1);

    return 0;

}


int uart_net_thread(int argc, char *argv[])

{
    int i=0;
    char data = '0';
    char buffer[4] = "";
//char a[7]="open\r\n";

int uart_read = uart_init((char *)"/dev/ttyS3");
 if(false == uart_read)
return -1;
if(false == set_uart_baudrate(uart_read,9600))
{
 printf("[JXF]set_uart_baudrate is failed\n");
return -1;
 } printf("[JXF]uart init is successful\n");


    while(1)
    {
        read(uart_read,&data,1);
        if(data == 'T'){
            for(i = 0;i <4;++i){
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0';
            }
         printf("%s\n",buffer);
memset(&buffer,0,sizeof(char[4]));
        }
// int j=0;
// while (j<20)
// {
// j++;
//  write(uart_read,&a,6);
//  }
//  printf("%s\n",a);

usleep(1000);
}
}
