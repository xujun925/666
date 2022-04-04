/***************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "uart_test.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int Uart_test::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Uart_test::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int Uart_test::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("uart_test",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Uart_test *Uart_test::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Uart_test *instance = new Uart_test(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Uart_test::Uart_test(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void Uart_test::run()
{
    char readbuf[10] {};
    char data='0';
 //   unsigned readlen = sizeof(readbuf) - 1;
    int ret = 0;
    double result = -1.0f;
    // open fd
    _fd = ::open((char *)"/dev/ttyS3", O_RDWR | O_NOCTTY);

    if (_fd < 0) {
        PX4_ERR("Error opening fd");
        //return;
    }

    // baudrate 115200, 8 bits, no parity, 1 stop bit
    unsigned speed = B115200;
    termios uart_config{};
    int termios_state{};

    tcgetattr(_fd, &uart_config);

    // clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    // set baud rate
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_ERR("CFG: %d ISPD", termios_state);
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        PX4_ERR("CFG: %d OSPD\n", termios_state);
    }

    if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
        PX4_ERR("baud %d ATTR", termios_state);
    }

    uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;			// 8-bit characters
    uart_config.c_cflag &= ~PARENB;			// no parity bit
    uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
    uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

    // setup for non-canonical mode
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    uart_config.c_oflag &= ~OPOST;

    // fetch bytes as they become available
    uart_config.c_cc[VMIN] = 1;
    uart_config.c_cc[VTIME] = 1;


	while (!should_exit()) {
//        senser_data.timestamp=hrt_absolute_time();
//        senser_data.max_distance=100;
//         _senser_data_pub.publish(senser_data);
ret = ::read(_fd, &data, 1);
if (ret < 0) {
    PX4_ERR("read err: %d", ret);
}
else
{
     if(data == 'F'){
         for(int i = 0;i <10;++i){
             read(_fd,&data,1);
             readbuf[i] = data;
             data = '0';
//             printf("readbuf[%d]:%c\n",i,readbuf[i]);
         }
//      printf("readbuf:%s\n",readbuf);
         result=(double)(readbuf[5]-'0')*100+(double)(readbuf[6]-'0')*10+(double)(readbuf[7]-'0')+(double)(readbuf[8]-'0')*0.1+(double)(readbuf[9]-'0')*0.01;
         printf("result:%lf\n",result);
         send_data.altitude=result;
         send_data.timestamp=hrt_absolute_time();
         _send_data_pub.publish(send_data);
//        senser_data.timestamp=hrt_absolute_time();
//        senser_data.max_distance=result;
//         _senser_data_pub.publish(senser_data);
     }

}
ret = ::write(_fd,&readbuf,10);
if (ret < 0) {
    PX4_ERR("write err: %d", ret);
}
else
{
  //  PX4_INFO("write buf:%f",result);
}
    memset(&readbuf,0,sizeof(char[10]));
        usleep(100000);
	}

}

void Uart_test::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int Uart_test::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int uart_test_main(int argc, char *argv[])
{
	return Uart_test::main(argc, argv);
}
