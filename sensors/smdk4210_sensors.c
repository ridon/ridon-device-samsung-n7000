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
#include <poll.h>
#include <sys/select.h>
#include <hardware/sensors.h>
#include <hardware/hardware.h>

#define LOG_TAG "smdk4210_sensors"
#include <utils/Log.h>

#include "smdk4210_sensors.h"

/*
 * Sensors list
 */

struct sensor_t smdk4210_sensors[] = {
	{ "K3DH Acceleration Sensor", "STMicroelectronics", 1, SENSOR_TYPE_ACCELEROMETER,
		SENSOR_TYPE_ACCELEROMETER, 2 * GRAVITY_EARTH, GRAVITY_EARTH / 90.0f / 8.0f, 0.23f, 20000, {}, },
	{ "AKM8975 Magnetic Sensor", "Asahi Kasei Microdevices", 1, SENSOR_TYPE_MAGNETIC_FIELD,
		SENSOR_TYPE_MAGNETIC_FIELD, 2000.0f, 1.0f / 16, 6.8f, 16667, {}, },
	{ "Orientation Sensor", "SMDK4210 Sensors", 1, SENSOR_TYPE_ORIENTATION,
		SENSOR_TYPE_ORIENTATION, 360.0f, 0.1f, 0.0f, 16667, {}, },
	{ "GP2A Light Sensor", "Sharp", 1, SENSOR_TYPE_LIGHT,
		SENSOR_TYPE_LIGHT, 10240.0f, 1.0f, 0.75f, 0, {}, },
	{ "GP2A Proximity Sensor", "Sharp", 1, SENSOR_TYPE_PROXIMITY,
		SENSOR_TYPE_PROXIMITY, 5.0f, 5.0f, 0.75f, 0, {}, },
	{ "K3G Gyroscope Sensor", "STMicroelectronics", 1, SENSOR_TYPE_GYROSCOPE,
		SENSOR_TYPE_GYROSCOPE, 500.0f * (3.1415926535f / 180.0f), (70.0f / 4000.0f) * (3.1415926535f / 180.0f), 6.1f, 1190, {}, },
	{ "BMP180 Pressure Sensor", "Bosch", 1, SENSOR_TYPE_PRESSURE,
		SENSOR_TYPE_PRESSURE, 1100.0f, 0.01f, 0.67f, 20000, {}, },
};

int smdk4210_sensors_count = sizeof(smdk4210_sensors) / sizeof(struct sensor_t);

struct smdk4210_sensors_handlers *smdk4210_sensors_handlers[] = {
	&k3dh,
	&akm8975,
	&orientation,
	&gp2a_light,
	&gp2a_proximity,
	&k3g,
	&bmp180,
};

int smdk4210_sensors_handlers_count = sizeof(smdk4210_sensors_handlers) /
	sizeof(struct smdk4210_sensors_handlers *);

/*
 * SMDK4210 Sensors
 */

int smdk4210_sensors_activate(struct sensors_poll_device_t *dev, int handle,
	int enabled)
{
	struct smdk4210_sensors_device *device;
	int i;

	ALOGD("%s(%p, %d, %d)", __func__, dev, handle, enabled);

	if (dev == NULL)
		return -EINVAL;

	device = (struct smdk4210_sensors_device *) dev;

	if (device->handlers == NULL || device->handlers_count <= 0)
		return -EINVAL;

	for (i = 0; i < device->handlers_count; i++) {
		if (device->handlers[i] == NULL)
			continue;

		if (device->handlers[i]->handle == handle) {
			if (enabled && device->handlers[i]->activate != NULL) {
				device->handlers[i]->needed |= SMDK4210_SENSORS_NEEDED_API;
				if (device->handlers[i]->needed == SMDK4210_SENSORS_NEEDED_API)
					return device->handlers[i]->activate(device->handlers[i]);
				else
					return 0;
			} else if (!enabled && device->handlers[i]->deactivate != NULL) {
				device->handlers[i]->needed &= ~SMDK4210_SENSORS_NEEDED_API;
				if (device->handlers[i]->needed == 0)
					return device->handlers[i]->deactivate(device->handlers[i]);
				else
					return 0;
			}
		}
	}

	return -1;
}

int smdk4210_sensors_set_delay(struct sensors_poll_device_t *dev, int handle,
	int64_t ns)
{
	struct smdk4210_sensors_device *device;
	int i;

	ALOGD("%s(%p, %d, %ld)", __func__, dev, handle, (long int) ns);

	if (dev == NULL)
		return -EINVAL;

	device = (struct smdk4210_sensors_device *) dev;

	if (device->handlers == NULL || device->handlers_count <= 0)
		return -EINVAL;

	for (i = 0; i < device->handlers_count; i++) {
		if (device->handlers[i] == NULL)
			continue;

		if (device->handlers[i]->handle == handle && device->handlers[i]->set_delay != NULL)
			return device->handlers[i]->set_delay(device->handlers[i], (long int) ns);
	}

	return 0;
}

int smdk4210_sensors_poll(struct sensors_poll_device_t *dev,
	struct sensors_event_t* data, int count)
{
	struct smdk4210_sensors_device *device;
	int i, j;
	int c, n;
	int poll_rc, rc;

//	ALOGD("%s(%p, %p, %d)", __func__, dev, data, count);

	if (dev == NULL)
		return -EINVAL;

	device = (struct smdk4210_sensors_device *) dev;

	if (device->handlers == NULL || device->handlers_count <= 0 ||
		device->poll_fds == NULL || device->poll_fds_count <= 0)
		return -EINVAL;

	n = 0;

	do {
		poll_rc = poll(device->poll_fds, device->poll_fds_count, n > 0 ? 0 : -1);
		if (poll_rc < 0)
			return -1;

		for (i = 0; i < device->poll_fds_count; i++) {
			if (!(device->poll_fds[i].revents & POLLIN))
				continue;

			for (j = 0; j < device->handlers_count; j++) {
				if (device->handlers[j] == NULL || device->handlers[j]->poll_fd != device->poll_fds[i].fd || device->handlers[j]->get_data == NULL)
					continue;

				rc = device->handlers[j]->get_data(device->handlers[j], &data[n]);
				if (rc < 0) {
					device->poll_fds[i].revents = 0;
					poll_rc = -1;
				} else {
					n++;
					count--;
				}
			}
		}
	} while ((poll_rc > 0 || n < 1) && count > 0);

	return n;
}

/*
 * Interface
 */

int smdk4210_sensors_close(hw_device_t *device)
{
	struct smdk4210_sensors_device *smdk4210_sensors_device;
	int i;

	ALOGD("%s(%p)", __func__, device);

	if (device == NULL)
		return -EINVAL;

	smdk4210_sensors_device = (struct smdk4210_sensors_device *) device;

	if (smdk4210_sensors_device->poll_fds != NULL)
		free(smdk4210_sensors_device->poll_fds);

	for (i = 0; i < smdk4210_sensors_device->handlers_count; i++) {
		if (smdk4210_sensors_device->handlers[i] == NULL || smdk4210_sensors_device->handlers[i]->deinit == NULL)
			continue;

		smdk4210_sensors_device->handlers[i]->deinit(smdk4210_sensors_device->handlers[i]);
	}

	free(device);

	return 0;
}

int smdk4210_sensors_open(const struct hw_module_t* module, const char *id,
	struct hw_device_t** device)
{
	struct smdk4210_sensors_device *smdk4210_sensors_device;
	int p, i;

	ALOGD("%s(%p, %s, %p)", __func__, module, id, device);

	if (module == NULL || device == NULL)
		return -EINVAL;

	smdk4210_sensors_device = (struct smdk4210_sensors_device *)
		calloc(1, sizeof(struct smdk4210_sensors_device));
	smdk4210_sensors_device->device.common.tag = HARDWARE_DEVICE_TAG;
	smdk4210_sensors_device->device.common.version = 0;
	smdk4210_sensors_device->device.common.module = (struct hw_module_t *) module;
	smdk4210_sensors_device->device.common.close = smdk4210_sensors_close;
	smdk4210_sensors_device->device.activate = smdk4210_sensors_activate;
	smdk4210_sensors_device->device.setDelay = smdk4210_sensors_set_delay;
	smdk4210_sensors_device->device.poll = smdk4210_sensors_poll;
	smdk4210_sensors_device->handlers = smdk4210_sensors_handlers;
	smdk4210_sensors_device->handlers_count = smdk4210_sensors_handlers_count;
	smdk4210_sensors_device->poll_fds = (struct pollfd *)
		calloc(1, smdk4210_sensors_handlers_count * sizeof(struct pollfd));

	p = 0;
	for (i = 0; i < smdk4210_sensors_handlers_count; i++) {
		if (smdk4210_sensors_handlers[i] == NULL || smdk4210_sensors_handlers[i]->init == NULL)
			continue;

		smdk4210_sensors_handlers[i]->init(smdk4210_sensors_handlers[i], smdk4210_sensors_device);
		if (smdk4210_sensors_handlers[i]->poll_fd >= 0) {
			smdk4210_sensors_device->poll_fds[p].fd = smdk4210_sensors_handlers[i]->poll_fd;
			smdk4210_sensors_device->poll_fds[p].events = POLLIN;
			p++;
		}
	}

	smdk4210_sensors_device->poll_fds_count = p;

	*device = &(smdk4210_sensors_device->device.common);

	return 0;
}

int smdk4210_sensors_get_sensors_list(struct sensors_module_t* module,
	const struct sensor_t **sensors_p)
{
	ALOGD("%s(%p, %p)", __func__, module, sensors_p);

	if (sensors_p == NULL)
		return -EINVAL;

	*sensors_p = smdk4210_sensors;
	return smdk4210_sensors_count;
}

struct hw_module_methods_t smdk4210_sensors_module_methods = {
	.open = smdk4210_sensors_open,
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
	.common = {
		.tag = HARDWARE_MODULE_TAG,
		.version_major = 1,
		.version_minor = 0,
		.id = SENSORS_HARDWARE_MODULE_ID,
		.name = "SMDK4210 Sensors",
		.author = "Paul Kocialkowski",
		.methods = &smdk4210_sensors_module_methods,
	},
	.get_sensors_list = smdk4210_sensors_get_sensors_list,
};
