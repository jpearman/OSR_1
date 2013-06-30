/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2012                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*        Module:     GyroLib.c                                                */
/*        Author:     James Pearman                                            */
/*        Created:    2 Oct 2012                                               */
/*                                                                             */
/*        Revisions:  V0.1                                                     */
/*                    V0.2 17 Feb 2013                                         */
/*                         Clear absolute angle and local parameters when      */
/*                         starting task.                                      */
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
/** @file    GyroLib.c
  * @brief   Library for the vex gyro
*//*---------------------------------------------------------------------------*/

// Stop recursive includes
#ifndef __GYROLIB__
#define __GYROLIB__

/** @brief Structure to hold global info for the gyro
 */
typedef struct {
    tSensors port;      ///< The analog port
    bool     valid;     ///< flag showing data is valid
    float    angle;     ///< raw gyro angle
    float    abs_angle; ///< Absolute gyro angle
    } gyroData;

static  gyroData    theGyro = {in1, false, 0.0, 0.0};

/*-----------------------------------------------------------------------------*/
/** @brief      Small debugging function                                       */
/** @param[in]  displayLine The line on the LCD to display debug data          */
/*-----------------------------------------------------------------------------*/

void
GyroDebug( int displayLine )
{
    string str;

    if( theGyro.valid )
        {
        // display current value
        sprintf(str,"Gyro %5.1f   ", theGyro.angle );
        displayLCDString(displayLine, 0, str);
        }
    else
        displayLCDString(displayLine, 0, "Init Gyro.." );
}

/*-----------------------------------------------------------------------------*/
/** @brief  Task that polls the Gyro and calculates the angle of rotation      */
/*-----------------------------------------------------------------------------*/

task GyroTask()
{
    int     gyro_value;
    int     gyro_error = 0;
    int     lastDriftGyro = 0;

    float   angle;
    float   old_angle = 0.0;
    float   delta_angle = 0.0;

    long    nSysTimeOffset;

    // Gyro readings invalid
    theGyro.valid = false;

    // clear absolute
    theGyro.abs_angle = 0;

    // Cause the gyro to reinitialize (a theory anyway)
    SensorType[theGyro.port] = sensorNone;

    // Wait 1/2 sec
    wait10Msec(50);

    // Gyro should be motionless here
    SensorType[theGyro.port] = sensorGyro;

    // Wait 1/2 sec
    wait10Msec(50);

    // What is the current system timer
    nSysTimeOffset = nSysTime;

    // loop forever
    while(true)
        {
        // get current gyro value (deg * 10)
        gyro_value = SensorValue[theGyro.port];

        // Filter drift when not moving
        if( (nSysTime - nSysTimeOffset) > 250 )
            {
            if( abs( gyro_value - lastDriftGyro ) < 3 )
                gyro_error += (lastDriftGyro - gyro_value);

            lastDriftGyro = gyro_value;

            nSysTimeOffset = nSysTime;
            }

        // Create float angle, remove offset
        angle = (gyro_value + gyro_error)  / 10.0;

        // normalize into the range 0 - 360
        if( angle < 0 )
            angle += 360;

        // store in struct for others
        theGyro.angle = angle;

        // work out change from last time
        delta_angle = angle - old_angle;
        old_angle   = angle;

        // fix rollover
        if(delta_angle > 180)
          delta_angle -= 360;
        if(delta_angle < -180)
          delta_angle += 360;

        // store absolute angle
        theGyro.abs_angle = theGyro.abs_angle + delta_angle;

        // We can use the angle
        theGyro.valid = true;

        // Delay
        wait1Msec( 20 );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Initialize the Gyro on the given port                          */
/** @param[in]  port The sensor port where the gyro is installed               */
/*-----------------------------------------------------------------------------*/

void
GyroInit( tSensors port  )
{
    theGyro.port = port;

    StartTask( GyroTask );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Reset the gyro                                                 */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Cause the gyro to be reinitialized by stopping and then restarting the
 *  polling task
 */

void
GyroReinit()
{
    StopTask( GyroTask );
    StartTask( GyroTask );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get the gyro angle                                             */
/** @returns    gyro angle in 0 - 360 deg range                                */
/*-----------------------------------------------------------------------------*/

float
GyroGetAngle()
{
    return( theGyro.angle );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get the absolute gyro angle                                    */
/** @returns    absolute gyro angle                                            */
/*-----------------------------------------------------------------------------*/

float
GyroGetAbsAngle()
{
    return( theGyro.abs_angle );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get the gyro valid flag                                        */
/** @returns    1 - gyro is valid 0 - gyro still initializing                  */
/*-----------------------------------------------------------------------------*/

bool
GyroGetValid()
{
    return( theGyro.valid );
}


#endif  //__GYROLIB__
