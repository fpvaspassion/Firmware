/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

/**
 * @file servo.cpp
 *
 * Servo output control
 *
 * @author Michail Lastovich <info@lastovich.ru>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>


/**
 * bottle_drop app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int servo_main(int argc, char *argv[]);

class Servo
{
public:
	/**
	 * Constructor
	 */
	Servo();

	/**
	 * Destructor, also kills task.
	 */
	~Servo();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display status.
	 */
	void		status();
	void		set_servo(int channel, float new_value);
/*
	void		open_bay();
	void		close_bay();
	void		drop();
	void		lock_release();
*/
private:
	bool	_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	int		_mavlink_fd;

	int		_command_sub;
	int		_wind_estimate_sub;
	struct vehicle_command_s	_command;

	struct vehicle_global_position_s _global_pos;
	map_projection_reference_s ref;

	orb_advert_t	_actuator_pub;
	struct actuator_controls_s _actuators;

	bool		_drop_approval;
	hrt_abstime	_doors_opened;
	hrt_abstime	_drop_time;

	float		_alt_clearance;

	struct position_s {
		double lat;	///< degrees
		double lon;	///< degrees
		float alt;	///< m
	} _target_position, _drop_position;

	enum DROP_STATE {
		DROP_STATE_INIT = 0,
		DROP_STATE_TARGET_VALID,
		DROP_STATE_TARGET_SET,
		DROP_STATE_BAY_OPEN,
		DROP_STATE_DROPPED,
		DROP_STATE_BAY_CLOSED
	} _drop_state;

	struct mission_s	_onboard_mission;
	orb_advert_t		_onboard_mission_pub;

	void		task_main();

	void		handle_command(struct vehicle_command_s *cmd);

	void		answer_command(struct vehicle_command_s *cmd, enum VEHICLE_CMD_RESULT result);


	/**
	 * Set the actuators
	 */
	int		actuators_publish();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);
};

namespace servo
{
Servo	*g_servo;
}

Servo::Servo() :

	_task_should_exit(false),
	_main_task(-1),
	_mavlink_fd(-1),
	_command_sub(-1),
	_wind_estimate_sub(-1),
	_command {},
	_global_pos {},
	ref {},
	_actuator_pub(-1),
	_actuators {},
	_drop_approval(false),
	_doors_opened(0),
	_drop_time(0),
	_alt_clearance(70.0f),
	_target_position {},
	_drop_position {},
	_drop_state(DROP_STATE_INIT),
	_onboard_mission {},
	_onboard_mission_pub(-1)
{
}

Servo::~Servo()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	servo::g_servo = nullptr;
}

int
Servo::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = task_spawn_cmd("servo",
				    SCHED_DEFAULT,
				    SCHED_PRIORITY_DEFAULT + 15,
				    2048,
				    (main_t)&Servo::task_main_trampoline,
				    nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


void
Servo::status()
{
	warnx("Servo active");
}

/*
void
Servo::open_bay()
{
	_actuators.control[0] = -1.0f;
	_actuators.control[1] = 1.0f;

	if (_doors_opened == 0) {
		_doors_opened = hrt_absolute_time();
	}
	warnx("open doors");
	actuators_publish();
	usleep(500 * 1000);
}

void
Servo::close_bay()
{
	// closed door and locked survival kit
	_actuators.control[0] = 1.0f;
	_actuators.control[1] = -1.0f;

	_doors_opened = 0;

	actuators_publish();

	// delay until the bay is closed
	usleep(500 * 1000);
}

void

Servo::drop()
{

	// update drop actuator, wait 0.5s until the doors are open before dropping
	hrt_abstime starttime = hrt_absolute_time();

	// force the door open if we have to
	if (_doors_opened == 0) {
		open_bay();
		warnx("bay not ready, forced open");
	}

	while (hrt_elapsed_time(&_doors_opened) < 500 * 1000 && hrt_elapsed_time(&starttime) < 2000000) {
		usleep(50000);
		warnx("delayed by door!");
	}

	_actuators.control[2] = 1.0f;

	_drop_time = hrt_absolute_time();
	actuators_publish();

	warnx("dropping now");

	// Give it time to drop
	usleep(1000 * 1000);
}

void
Servo::lock_release()
{
	_actuators.control[2] = -1.0f;
	actuators_publish();

	warnx("closing release");
}

*/

void

Servo::set_servo(int channel, float new_value)
{
	_actuators.control[channel] = new_value;

	warnx("set servo state");
	actuators_publish();

	usleep(500 * 1000);
}

int
Servo::actuators_publish()
{
	_actuators.timestamp = hrt_absolute_time();

	// lazily publish _actuators only once available
	if (_actuator_pub > 0) {
		return orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &_actuators);

	} else {
		_actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators);
		if (_actuator_pub > 0) {
			return OK;
		} else {
			return -1;
		}
	}
}

void
Servo::task_main()
{

	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(_mavlink_fd, "[servo] started");

	_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	_wind_estimate_sub = orb_subscribe(ORB_ID(wind_estimate));

	bool updated = false;

	int vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	unsigned counter = 0;

	// wakeup source(s)
	struct pollfd fds[1];

	// Setup of loop
	fds[0].fd = _command_sub;
	fds[0].events = POLLIN;

	// Whatever state the bay is in, we want it closed on startup
/*
	lock_release();
	close_bay();
*/

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* vehicle commands updated */
		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
			handle_command(&_command);
		}

		orb_check(vehicle_global_position_sub, &updated);
		if (updated) {
			/* copy global position */
			orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &_global_pos);
		}

		if (_global_pos.timestamp == 0) {
			continue;
		}

		const unsigned sleeptime_us = 9500;

		counter++;

		// update_actuators();

		// run at roughly 100 Hz
		usleep(sleeptime_us);


		}


	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}

void
Servo::handle_command(struct vehicle_command_s *cmd)
{
	switch (cmd->command) {
	case VEHICLE_CMD_CUSTOM_0:
		/*
		 * param1 channel
		 * param2 new PWM value for servo
		 */
		set_servo(cmd->param1, cmd->param2);
		mavlink_log_info(_mavlink_fd, "#audio: new value is set");

		answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	default:
		break;
	}
}

void
Servo::answer_command(struct vehicle_command_s *cmd, enum VEHICLE_CMD_RESULT result)
{
	switch (result) {
	case VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case VEHICLE_CMD_RESULT_DENIED:
		mavlink_log_critical(_mavlink_fd, "#audio: command denied: %u", cmd->command);
		break;

	case VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(_mavlink_fd, "#audio: command failed: %u", cmd->command);
		break;

	case VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		mavlink_log_critical(_mavlink_fd, "#audio: command temporarily rejected: %u", cmd->command);
		break;

	case VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(_mavlink_fd, "#audio: command unsupported: %u", cmd->command);
		break;

	default:
		break;
	}
}

void
Servo::task_main_trampoline(int argc, char *argv[])
{
	servo::g_servo->task_main();
}

static void usage()
{
	errx(1, "usage: servo {start|set|stop|status}");
}

int servo_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (servo::g_servo != nullptr) {
			errx(1, "already running");
		}

		servo::g_servo = new Servo;

		if (servo::g_servo == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != servo::g_servo->start()) {
			delete servo::g_servo;
			servo::g_servo = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (servo::g_servo == nullptr) {
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete servo::g_servo;
		servo::g_servo = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		servo::g_servo->status();

	} else if (!strcmp(argv[1], "set")) {
		servo::g_servo->set_servo(1,100.0);

	} else {
		usage();
	}

	return 0;
}
