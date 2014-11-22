/*
 * Copyright (C) 2013 Paul Kocialkowski <contact@paulk.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>

#include <hardware/sensors.h>
#include <hardware/hardware.h>

#define LOG_TAG "smdk4210_sensors"
#include <utils/Log.h>

#include "smdk4210_sensors.h"
#include "k3dh.h"

struct k3dh_data {
	struct smdk4210_sensors_handlers *orientation_sensor;

	long int delay;
	int device_fd;
	int uinput_fd;

	pthread_t thread;
	pthread_mutex_t mutex;
	int thread_continue;
};

void *k3dh_thread(void *thread_data)
{
	struct smdk4210_sensors_handlers *handlers = NULL;
	struct k3dh_data *data = NULL;
	struct input_event event;
	struct timeval time;
	struct k3dh_acceldata acceleration_data;
	long int before, after;
	int diff;
	int device_fd;
	int uinput_fd;
	int rc;

	if (thread_data == NULL)
		return NULL;

	handlers = (struct smdk4210_sensors_handlers *) thread_data;
	if (handlers->data == NULL)
		return NULL;

	data = (struct k3dh_data *) handlers->data;

	device_fd = data->device_fd;
	if (device_fd < 0)
		return NULL;

	uinput_fd = data->uinput_fd;
	if (uinput_fd < 0)
		return NULL;

	while (data->thread_continue) {
		pthread_mutex_lock(&data->mutex);
		if (!data->thread_continue)
			break;

		while (handlers->activated) {
			gettimeofday(&time, NULL);
			before = timestamp(&time);

			memset(&acceleration_data, 0, sizeof(acceleration_data));

			rc = ioctl(device_fd, K3DH_IOCTL_READ_ACCEL_XYZ, &acceleration_data);
			if (rc < 0) {
				ALOGE("%s: Unable to get k3dh data", __func__);
				return NULL;
			}

			input_event_set(&event, EV_REL, REL_X, (int) (acceleration_data.x * 1000));
			write(uinput_fd, &event, sizeof(event));
			input_event_set(&event, EV_REL, REL_Y, (int) (acceleration_data.y * 1000));
			write(uinput_fd, &event, sizeof(event));
			input_event_set(&event, EV_REL, REL_Z, (int) (acceleration_data.z * 1000));
			write(uinput_fd, &event, sizeof(event));
			input_event_set(&event, EV_SYN, 0, 0);
			write(uinput_fd, &event, sizeof(event));

next:
			gettimeofday(&time, NULL);
			after = timestamp(&time);

			diff = (int) (data->delay - (after - before)) / 1000;
			if (diff <= 0)
				continue;

			usleep(diff);
		}
	}
	return NULL;
}

int k3dh_init(struct smdk4210_sensors_handlers *handlers,
	struct smdk4210_sensors_device *device)
{
	struct k3dh_data *data = NULL;
	pthread_attr_t thread_attr;
	int device_fd = -1;
	int uinput_fd = -1;
	int input_fd = -1;
	int rc;
	int i;

	ALOGD("%s(%p, %p)", __func__, handlers, device);

	if (handlers == NULL || device == NULL)
		return -EINVAL;

	data = (struct k3dh_data *) calloc(1, sizeof(struct k3dh_data));

	for (i = 0; i < device->handlers_count; i++) {
		if (device->handlers[i] == NULL)
			continue;

		if (device->handlers[i]->handle == SENSOR_TYPE_ORIENTATION)
			data->orientation_sensor = device->handlers[i];
	}

	device_fd = open("/dev/accelerometer", O_RDONLY);
	if (device_fd < 0) {
		ALOGE("%s: Unable to open device", __func__);
		goto error;
	}

	uinput_fd = uinput_rel_create("acceleration");
	if (uinput_fd < 0) {
		ALOGD("%s: Unable to create uinput", __func__);
		goto error;
	}

	input_fd = input_open("acceleration");
	if (input_fd < 0) {
		ALOGE("%s: Unable to open acceleration input", __func__);
		goto error;
	}

	data->thread_continue = 1;

	pthread_mutex_init(&data->mutex, NULL);
	pthread_mutex_lock(&data->mutex);

	pthread_attr_init(&thread_attr);
	pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);

	rc = pthread_create(&data->thread, &thread_attr, k3dh_thread, (void *) handlers);
	if (rc < 0) {
		ALOGE("%s: Unable to create k3dh thread", __func__);
		pthread_mutex_destroy(&data->mutex);
		goto error;
	}

	data->device_fd = device_fd;
	data->uinput_fd = uinput_fd;
	handlers->poll_fd = input_fd;
	handlers->data = (void *) data;

	return 0;

error:
	if (data != NULL)
		free(data);

	if (uinput_fd >= 0)
		close(uinput_fd);

	if (input_fd >= 0)
		close(input_fd);

	if (device_fd >= 0)
		close(device_fd);

	handlers->poll_fd = -1;
	handlers->data = NULL;

	return -1;
}

int k3dh_deinit(struct smdk4210_sensors_handlers *handlers)
{
	struct k3dh_data *data = NULL;

	ALOGD("%s(%p)", __func__, handlers);

	if (handlers == NULL || handlers->data == NULL)
		return -EINVAL;

	data = (struct k3dh_data *) handlers->data;

	handlers->activated = 0;
	data->thread_continue = 0;
	pthread_mutex_unlock(&data->mutex);

	pthread_mutex_destroy(&data->mutex);

	if (data->uinput_fd >= 0) {
		uinput_destroy(data->uinput_fd);
		close(data->uinput_fd);
	}
	data->uinput_fd = -1;

	if (handlers->poll_fd >= 0)
		close(handlers->poll_fd);
	handlers->poll_fd = -1;

	if (data->device_fd >= 0)
		close(data->device_fd);
	data->device_fd = -1;

	free(handlers->data);
	handlers->data = NULL;

	return 0;
}

int k3dh_activate(struct smdk4210_sensors_handlers *handlers)
{
	struct k3dh_data *data;
	int device_fd;
	int enable;
	int rc;

	ALOGD("%s(%p)", __func__, handlers);

	if (handlers == NULL || handlers->data == NULL)
		return -EINVAL;

	if (handlers->activated == 1)
		return 0;

	data = (struct k3dh_data *) handlers->data;

	device_fd = data->device_fd;
	if (device_fd < 0)
		return -1;

	enable = 1;
	rc = ioctl(device_fd, K3DH_IOCTL_SET_ENABLE, &enable);
	if (rc < 0) {
		ALOGE("%s: Unable to set k3dh enable", __func__);
		return -1;
	}
	
	handlers->activated = 1;
	pthread_mutex_unlock(&data->mutex);

	return 0;
}

int k3dh_deactivate(struct smdk4210_sensors_handlers *handlers)
{
	struct k3dh_data *data;
	int device_fd;
	int enable;
	int rc;

	ALOGD("%s(%p)", __func__, handlers);

	if (handlers == NULL || handlers->data == NULL)
		return -EINVAL;

	if (handlers->activated == 0)
		return 0;

	data = (struct k3dh_data *) handlers->data;

	device_fd = data->device_fd;
	if (device_fd < 0)
		return -1;

	enable = 0;
	rc = ioctl(device_fd, K3DH_IOCTL_SET_ENABLE, &enable);
	if (rc < 0) {
		ALOGE("%s: Unable to set k3dh enable", __func__);
		return -1;
	}

	handlers->activated = 0;

	return 0;
}

int k3dh_set_delay(struct smdk4210_sensors_handlers *handlers, long int delay)
{
	struct k3dh_data *data;
	int64_t d;
	int device_fd;
	int rc;

	ALOGD("%s(%p, %ld)", __func__, handlers, delay);

	if (handlers == NULL || handlers->data == NULL)
		return -EINVAL;

	data = (struct k3dh_data *) handlers->data;

	device_fd = data->device_fd;
	if (device_fd < 0)
		return -1;

	d = (int64_t) delay;
	rc = ioctl(device_fd, K3DH_IOCTL_SET_DELAY, &d);
	if (rc < 0) {
		ALOGE("%s: Unable to set k3dh delay", __func__);
		return -1;
	}

	data->delay = delay;

	return 0;
}

float k3dh_convert(int value)
{
	return (float) (value / 1000.0f) * GRAVITY_EARTH * 0.7031f / 90.0f / 8.0f;
}

int k3dh_get_data(struct smdk4210_sensors_handlers *handlers,
	struct sensors_event_t *event)
{
	struct k3dh_data *data;
	struct input_event input_event;
	int input_fd;
	int rc;

//	ALOGD("%s(%p, %p)", __func__, handlers, event);

	if (handlers == NULL || handlers->data == NULL || event == NULL)
		return -EINVAL;

	data = (struct k3dh_data *) handlers->data;

	input_fd = handlers->poll_fd;
	if (input_fd < 0)
		return -1;

	memset(event, 0, sizeof(struct sensors_event_t));
	event->version = sizeof(struct sensors_event_t);
	event->sensor = handlers->handle;
	event->type = handlers->handle;

	event->magnetic.status = SENSOR_STATUS_ACCURACY_MEDIUM;

	do {
		rc = read(input_fd, &input_event, sizeof(input_event));
		if (rc < (int) sizeof(input_event))
			break;

		if (input_event.type == EV_REL) {
			switch (input_event.code) {
				case REL_X:
					event->acceleration.y = k3dh_convert(input_event.value * -1);
					break;
				case REL_Y:
					event->acceleration.x = k3dh_convert(input_event.value * -1);
					break;
				case REL_Z:
					event->acceleration.z = k3dh_convert(input_event.value);
					break;
				default:
					continue;
			}
		} else if (input_event.type == EV_SYN) {
			if (input_event.code == SYN_REPORT)
				event->timestamp = input_timestamp(&input_event);
		}
	} while (input_event.type != EV_SYN);

	if (data->orientation_sensor != NULL)
		orientation_fill(data->orientation_sensor, &event->acceleration, NULL);

	return 0;
}

struct smdk4210_sensors_handlers k3dh = {
	.name = "K3DH",
	.handle = SENSOR_TYPE_ACCELEROMETER,
	.init = k3dh_init,
	.deinit = k3dh_deinit,
	.activate = k3dh_activate,
	.deactivate = k3dh_deactivate,
	.set_delay = k3dh_set_delay,
	.get_data = k3dh_get_data,
	.activated = 0,
	.needed = 0,
	.poll_fd = -1,
	.data = NULL,
};
