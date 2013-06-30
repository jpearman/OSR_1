/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2012                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     PidLib.c                                                     */
/*    Author:     James Pearman                                                */
/*    Created:    24 Oct 2012                                                  */
/*                                                                             */
/*    Revisions:                                                               */
/*                30 Oct 2012 - Added Kbias                                    */
/*      V1.00     30 June 2013 - Initial release                               */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    This file is part of OSR_1 (open source robot 1)                         */
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail at jbpearman_at_mac_dot_com                           */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/** @file    PidLib.c
  * @brief   Library to handle closed loop PID calculations
*//*---------------------------------------------------------------------------*/

#ifndef __PIDLIB__
#define __PIDLIB__

// Version 1.02
#define kPidLibVersion   102

/** @brief Structure to hold all data for one instance of a PID controller
 */
typedef struct {
    // Turn on or off the control loop
    short        enabled;

    // PID constants, Kbias is used to compensate for gravity or similar
    float        Kp;
    float        Ki;
    float        Kd;
    float        Kbias;

    // working variables
    float        error;
    float        last_error;
    float        integral;
    float        integral_limit;
    float        derivative;
    float        error_threshold;

    // output
    float        drive;
    short        drive_raw;
    short        drive_cmd;

    tSensors     sensor_port;
    short        sensor_reverse;
    TSensorTypes sensor_type;
    long         sensor_value;

    long         target_value;
    } pidController;


/** @brief Allow 4 pid controllers
 */
#define MAX_PID               4

// static storage - we have no malloc
static  pidController   _pidControllers[ MAX_PID ];
static  short           nextPidControllerPtr = 0;

// lookup table to linearize control
#define PIDLIB_LUT_SIZE     128
#define PIDLIB_LUT_FACTOR  20.0
#define PIDLIB_LUT_OFFSET    10
static  short   PidDriveLut[PIDLIB_LUT_SIZE];
#define _LinearizeDrive( x )    PidDriveLut[abs(x)] * sgn(x)
void    PidControllerMakeLut();

// This causes the motor never to be given more than a 0.25 drive command due to integral
#define PIDLIB_INTEGRAL_DRIVE_MAX   0.25

/*-----------------------------------------------------------------------------*/
/** @brief      Initialize the PID controller                                  */
/** @param[in]  Kp proportional constant                                       */
/** @param[in]  Ki integral constant                                           */
/** @param[in]  Kd derivative constant                                         */
/** @param[in]  port senspr port for feedback                                  */
/** @param[in]  sensor_reverse reserse raw sensor value before use             */
/** @returns    A pointer to a pidController structure                         */
/*-----------------------------------------------------------------------------*/

pidController *
PidControllerInit( float Kp, float Ki, float Kd, tSensors port, short sensor_reverse = 0 )
{
    pidController   *p;

    if( nextPidControllerPtr == MAX_PID )
        return(NULL);

    p = (pidController *)&_pidControllers[ nextPidControllerPtr++ ];

    // pid constants
    p->Kp    = Kp;
    p->Ki    = Ki;
    p->Kd    = Kd;
    p->Kbias = 0.0;

    // zero out working variables
    p->error           = 0;
    p->last_error      = 0;
    p->integral        = 0;
    p->derivative      = 0;
    p->drive           = 0.0;
    p->drive_cmd       = 0;
    if(Ki != 0)
        p->integral_limit  = (PIDLIB_INTEGRAL_DRIVE_MAX / Ki);
    else
        p->integral_limit  = 0;

    p->error_threshold = 10;

    // sensor port
    p->sensor_port     = port;
    p->sensor_reverse  = sensor_reverse;
    p->sensor_type     = SensorType[ port ];
    p->sensor_value    = 0;

    p->target_value    = 0;

    // We need a valid sensor for pid control, pot or encoder
    if( ( p->sensor_type == sensorPotentiometer ) ||
        ( p->sensor_type == sensorQuadEncoder ) ||
        ( p->sensor_type == sensorQuadEncoderOnI2CPort )
      )
        p->enabled    = 1;
    else
        p->enabled    = 0;

    PidControllerMakeLut();

    return(p);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Initialize the PID controller with bias                        */
/** @param[in]  Kp proportional constant                                       */
/** @param[in]  Ki integral constant                                           */
/** @param[in]  Kd derivative constant                                         */
/** @param[in]  Kbias bias constant                                            */
/** @param[in]  port senspr port for feedback                                  */
/** @param[in]  sensor_reverse reserse raw sensor value before use             */
/** @returns    A pointer to a pidController structure                         */
/*-----------------------------------------------------------------------------*/

pidController *
PidControllerInit( float Kp, float Ki, float Kd, float Kbias, tSensors port, short sensor_reverse = 0 )
{
    pidController   *p;
    p = PidControllerInit( Kp, Ki, Kd, port, sensor_reverse );
    if( p != NULL)
        p->Kbias = Kbias;

    return(p);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Update the process command                                     */
/** @param[in]  p Pointer to a pid control structure                           */
/** @returns    drive cmd or 0 if error                                        */
/*-----------------------------------------------------------------------------*/

short
PidControllerUpdate( pidController *p )
{
    if( p == NULL )
        return(0);

    if( p->enabled )
        {
        // check for sensor port
        // otherwise externally calculated error
        if( p->sensor_port >= 0 )
            {
#ifdef _Target_Emulator_
            int inc = p->drive_cmd / 8;
            p->sensor_value += inc;
#else
            // Get raw position value, may be pot or encoder
            p->sensor_value = SensorValue[ p->sensor_port ];
#endif

            // A reversed sensor ?
            if( p->sensor_reverse )
                {
                if( p->sensor_type == sensorPotentiometer )
                    // reverse pot
                    p->sensor_value = 4095 - p->sensor_value;
                else
                    // reverse encoder
                    p->sensor_value = -p->sensor_value;
                }

            p->error = p->target_value - p->sensor_value;
            }

        // force error to 0 if below threshold
        if( abs(p->error) < p->error_threshold )
            p->error = 0;

        // integral accumulation
        if( p->Ki != 0 )
            {
            p->integral += p->error;

            // limit to avoid windup
            if( abs( p->integral ) > p->integral_limit )
                p->integral = sgn(p->integral) * p->integral_limit;
            }
        else
            p->integral = 0;

        // derivative
        p->derivative = p->error - p->last_error;
        p->last_error = p->error;

        // calculate drive - no delta T in this version
        p->drive = (p->Kp * p->error) + (p->Ki * p->integral) + (p->Kd * p->derivative) + p->Kbias;

        // drive should be in the range +/- 1.0
        if( abs( p->drive ) > 1.0 )
            p->drive = sgn(p->drive);

        // final motor output
        p->drive_raw = p->drive * 127.0;
        }

    else
        {
        // Disabled - all 0
        p->error      = 0;
        p->last_error = 0;
        p->integral   = 0;
        p->derivative = 0;
        p->drive      = 0.0;
        p->drive_raw  = 0;
        }

    // linearize - be careful this is a macro
    p->drive_cmd = _LinearizeDrive( p->drive_raw );

    // return the thing we are really interested in
    return( p->drive_cmd );
}


/*-----------------------------------------------------------------------------*/
/** @brief      Create a power based lut for the PID controller                */
/*-----------------------------------------------------------------------------*/

void
PidControllerMakeLut()
{
    int   i;
    float x;

    for(i=0;i<PIDLIB_LUT_SIZE;i++)
        {
        // check for valid power base
        if( PIDLIB_LUT_FACTOR > 1 )
            {
            x = pow( PIDLIB_LUT_FACTOR, (float)i / (float)(PIDLIB_LUT_SIZE-1) );

            if(i >= (PIDLIB_LUT_OFFSET/2))
               PidDriveLut[i] = (((x - 1.0) / (PIDLIB_LUT_FACTOR - 1.0)) * (PIDLIB_LUT_SIZE-1-PIDLIB_LUT_OFFSET)) + PIDLIB_LUT_OFFSET;
            else
               PidDriveLut[i] = i * 2;
            }
        else
            {
            // Linear
            PidDriveLut[i] = i;
            }
        }
}

#endif  // __PIDLIB__
