/**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#include "ev3api.h"
#include "app.h"

//#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static const int left_motor = EV3_PORT_A;
static const int right_motor = EV3_PORT_B;
static const int arm_motor = EV3_PORT_C;

static const int color_sensor = EV3_PORT_1;
static const int ultrasonic_sensor = EV3_PORT_2;
static const int touch_sensor0 = EV3_PORT_3;
static const int touch_sensor1 = EV3_PORT_4;

#define ERR_CHECK(err)  \
do {    \
    if ((err) != 0) {   \
        syslog(LOG_NOTICE, "ERROR: %s %d err=%d", __FUNCTION__, __LINE__, (err));   \
    }   \
} while (0)


void main_task(intptr_t unused) {
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_sensor_config(ultrasonic_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(touch_sensor0, TOUCH_SENSOR);
    ev3_sensor_config(touch_sensor1, TOUCH_SENSOR);
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(arm_motor, LARGE_MOTOR);
  
    syslog(LOG_NOTICE, "#### motor control start");
    while(1) {
        tslp_tsk(100000); /* 100msec */
    }
}
