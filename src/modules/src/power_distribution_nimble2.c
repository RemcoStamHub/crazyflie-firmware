/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_nimble.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"
#include "math.h"

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 32767);
  motorsSetRatio(MOTOR_M3, 32767);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)
{
  static float gam_max = 60/180*3.1416;
  static int16_t roll_trim = 0; //-6000; // 100% = 32767
  static int16_t pitch_trim = 0; //-10000; // 100% = 32767
  static int16_t yaw_trim = 0; //3000; // 100% = 32767
  static int16_t act_max = 32767;
  
  static float gam_in, gam, servo_max, yaw_scale;
  static int32_t croll, cpitch, cyaw;

  croll = control->roll;
  cpitch = control->pitch ;
  cyaw = control->yaw;

  // Actuator deflection for the given yaw command
  gam_in = gam_max*(float)cyaw/act_max;

  // check, if saturation will occur, if so, saturate the yaw command
  servo_max = fabs(yaw_trim) + fabs(control->yaw) + 0.4*fabs(cos(gam_in)*cpitch) + 0.2*fabs(sin(gam_in)*croll);

  if (servo_max > act_max)
  {
    if (gam_in > 0) gam = gam_in - (servo_max - act_max)/act_max*gam_max;
    else gam = gam_in + (servo_max - act_max)/act_max*gam_max;
  }
  else
  {
    gam = gam_in;
  }

  if (gam_in == 0) yaw_scale=1;
  else yaw_scale = gam/gam_in;

  motorPower.m2 = limitThrust(act_max + 0.2f * croll * sinf(gam) + 0.4f * cpitch * cosf(gam) + yaw_scale*cyaw + yaw_trim + pitch_trim ); // left servo
  motorPower.m3 = limitThrust(act_max - 0.2f * croll * sinf(gam) - 0.4f * cpitch * cosf(gam) + yaw_scale*cyaw + yaw_trim - pitch_trim ); // right servo
  motorPower.m4 = limitThrust( 0.5f * croll * cosf(gam) + 0.25f * cpitch * sinf(gam) + control->thrust * (1 + roll_trim / act_max) ); // left motor
  motorPower.m1 = limitThrust(-0.5f * croll * cosf(gam) - 0.25f * cpitch * sinf(gam) + control->thrust * (1 - roll_trim / act_max) ); // right motor
  
  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
