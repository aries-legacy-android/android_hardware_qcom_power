/* The modem temperature service is not shipped in source form and its client
 * requires the QMI framework.  Probing reports "not present" so the sensor
 * setup falls back to the local tsens/bcl/adc sources, which are unaffected.
 */

#include <stddef.h>

#include "thermal.h"

int qmi_ts_setup(sensor_setting_t *settings, sensor_t *sensor)
{
	(void)settings;
	(void)sensor;
	return 0;
}

void qmi_ts_shutdown(sensor_setting_t *setting)
{
	(void)setting;
}

int qmi_ts_get_temperature(sensor_setting_t *setting)
{
	(void)setting;
	return 0;
}

void qmi_ts_interrupt_wait(sensor_setting_t *setting)
{
	(void)setting;
}

void qmi_ts_update_thresholds(sensor_setting_t *setting, int threshold_type,
			      int level)
{
	(void)setting;
	(void)threshold_type;
	(void)level;
}
