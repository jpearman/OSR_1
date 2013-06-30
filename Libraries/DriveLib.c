/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*        Module:     DriveLib.c                                               */
/*        Author:     James Pearman                                            */
/*        Created:    3 Oct 2012                                               */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00     30 June 2013 - Initial release                     */
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
/** @file    DriveLib.c
  * @brief   Standardized arcade and mecanum drive
*//*---------------------------------------------------------------------------*/

// Stop recursive includes
#ifndef __DRIVELIB__
#define __DRIVELIB__

#ifndef __CTLLIB__
#include "Libraries\CtlLib.c"
#endif

/** @brief Different types of drive styles
 */
typedef enum {
    Arcade2Motor = 0,           ///< arcade drive with two motors
    Arcade4Motor,               ///< arcade drive with 4 motors
    Mecanum4Motor,              ///< mecanum drive with 4 motors
    Mecanum4MotorFieldCentric   ///< mecanum drive using gyro
} driveType;

/** @brief  Four wheel (per side) maximium
 */
#define kMaxWheelCount          4


/** @brief Structure holds all drive related data together
 */
typedef struct {
    /** Drive type
     */
    driveType   type;

    /** Number of total wheels
     */
    int         wheelCount;

    /** Storage for right motors
     */
    tMotor      rightWheels[kMaxWheelCount];
    /** Storage for left motors
     */
    tMotor      leftWheels[kMaxWheelCount];

    /** controller
     */
    vexrtController controller;

    // for gyro driving
    int         forward_req;
    int         right_req;
    int         turn_req;
    float       keep_angle;
} DriveSystem;

static  DriveSystem theDrive;

/*-----------------------------------------------------------------------------*/
/** @brief      Initialial drive system with two motors - arcade drive only    */
/** @param[in]  type control type, Arcade2Motor etc.                           */
/** @param[in]  lf left motor                                                  */
/** @param[in]  rf right motor                                                 */
/*-----------------------------------------------------------------------------*/

void
DriveSystemInit( driveType type, tMotor lf, tMotor rf )
{
    theDrive.type = type;
    theDrive.wheelCount = 2;

    theDrive.leftWheels[0]  = lf;
    theDrive.rightWheels[0] = rf;

    // Init controller
    ControllerInit( &theDrive.controller );
    // default control
    ControllerAttachControls( &theDrive.controller, Ch3, Ch4, Ch1 );

    // Clear gyro variables
    theDrive.forward_req = 0;
    theDrive.right_req   = 0;
    theDrive.turn_req    = 0;
    theDrive.keep_angle  = 0.0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Initialial drive system with four motors                       */
/** @param[in]  type control type, Arcade4Motor etc.                           */
/** @param[in]  lf left motor front                                            */
/** @param[in]  lr left motor rear                                             */
/** @param[in]  rf right motor front                                           */
/** @param[in]  rr right motor rear                                            */
/*-----------------------------------------------------------------------------*/

void
DriveSystemInit( driveType type, tMotor lf, tMotor lr, tMotor rf, tMotor rr )
{
    theDrive.type = type;
    theDrive.wheelCount = 4;

    theDrive.leftWheels[0]  = lf;
    theDrive.leftWheels[1]  = lr;
    theDrive.rightWheels[0] = rf;
    theDrive.rightWheels[1] = rr;

    // Init controller
    ControllerInit( &theDrive.controller );
    // default control
    ControllerAttachControls( &theDrive.controller, Ch3, Ch4, Ch1 );

    // Clear gyro variables
    theDrive.forward_req = 0;
    theDrive.right_req   = 0;
    theDrive.turn_req    = 0;
    theDrive.keep_angle  = 0.0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Switch drive types                                             */
/** @param[in]  type control type, Arcade4Motor etc.                           */
/*-----------------------------------------------------------------------------*/

void
DriveSystemChangeType( driveType type )
{
    // only allow certain changes
    // 2 motor is fixed
    if( theDrive.type == Arcade2Motor )
        return;

    // dont allow change to 2 motor
    if( type == Arcade2Motor )
        return;

    theDrive.type = type;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Attach controls to the drive                                   */
/** @param[in]  c1 A control index (Ch1, Btn8U etc)                            */
/** @param[in]  c2 A control index (optional)                                  */
/** @param[in]  c3 A control index (optional)                                  */
/** @param[in]  c4 A control index (optional)                                  */
/** @param[in]  c5 A control index (optional)                                  */
/** @param[in]  c6 A control index (optional)                                  */
/*-----------------------------------------------------------------------------*/
/** @note
 *  Control can either be with joysticks or buttons or any combinations
 */

void
DriveSystemAttachControls( short c1, short c2 = (-1), short c3 = (-1), short c4 = (-1), short c5 = (-1), short c6 = (-1))
{
    ControllerAttachControls( &theDrive.controller, c1, c2, c3, c4, c5, c6);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Set the drive controller channel scaling                       */
/** @param[in]  channel The control channel                                    */
/** @param[in]  scale The scale factor (0.0 to 1.0)                            */
/*-----------------------------------------------------------------------------*/
/** @details
 *  change the scaling if we are using buttons for control
 *  for example, if we want +/- 64 to be maximum control values then set scale
 *  as 0.5
 */

void
DriveSystemSetChannelScale( int channel, float scale )
{
    if( (scale < 0.0) || (scale > 1.0))
        return;

    if( channel == 0 ) {
        ControllerSetChannelScale( &theDrive.controller, CTL00_P, scale );
        ControllerSetChannelScale( &theDrive.controller, CTL00_N, scale );
        }
    if( channel == 1 ) {
        ControllerSetChannelScale( &theDrive.controller, CTL01_P, scale );
        ControllerSetChannelScale( &theDrive.controller, CTL01_N, scale );
        }
    if( channel == 2 ) {
        ControllerSetChannelScale( &theDrive.controller, CTL02_P, scale );
        ControllerSetChannelScale( &theDrive.controller, CTL02_N, scale );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get the drive control values                                   */
/** @param[in]  ctl0 Pointer to storage for first control variable             */
/** @param[in]  ctl1 Pointer to storage for second control variable            */
/** @param[in]  ctl2 Pointer to storage for third control variable             */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Get upto three control values (2 used for arcade, 3 for mecanum)
 */
void
DriveSystemGetControlValues( int *ctl0, int *ctl1, int *ctl2 )
{
    ControllerGetControlValues( &theDrive.controller, ctl0, ctl1, ctl2 );
}


/*-----------------------------------------------------------------------------*/
/*                                                                             */

#ifndef __GYROLIB__
// If gyro lib was not included then GyroGetAngle will not exist
// create a dummy version
#warning("Gyro library is not included, no field centric control")
float
GyroGetAngle()
{
    return(0.0);
}
float
GyroGetAbsAngle()
{
    return(0.0);
}
#endif

/*-----------------------------------------------------------------------------*/
/** @brief      control the mecanum drive                                      */
/** @param[in]  forward The forward power/speed (-127 to 127)                  */
/** @param[in]  turn The turning power/speed (-127 to 127)                     */
/** @param[in]  right The strafing power/speed (-127 to 127)                   */
/*-----------------------------------------------------------------------------*/

void
DriveSystemMecanumDrive( int forward, int turn, int right )
{
    long drive_l_front;
    long drive_l_back;
    long drive_r_front;
    long drive_r_back;

    // Set drive
    drive_l_front = forward + turn + right;
    drive_l_back  = forward + turn - right;

    drive_r_front = forward - turn - right;
    drive_r_back  = forward - turn + right;

    // normalize drive so max is 127 if any drive is over 127
    int max = abs(drive_l_front);
    if (abs(drive_l_back)  > max)
        max = abs(drive_l_back);
    if (abs(drive_r_back)  > max)
        max = abs(drive_r_back);
    if (abs(drive_r_front) > max)
        max = abs(drive_r_front);
    if (max>127) {
        drive_l_front = 127 * drive_l_front / max;
        drive_l_back  = 127 * drive_l_back  / max;
        drive_r_back  = 127 * drive_r_back  / max;
        drive_r_front = 127 * drive_r_front / max;
    }

    // Get control of motors
    MotorGetSemaphore();

    // Send to motors
    // left drive
    SetMotor( theDrive.leftWheels[0], drive_l_front);
    SetMotor( theDrive.leftWheels[1], drive_l_back);

    // right drive
    SetMotor( theDrive.rightWheels[0], drive_r_front);
    SetMotor( theDrive.rightWheels[1], drive_r_back);

    // release control of motors
    MotorReleaseSemaphore();
}

/*-----------------------------------------------------------------------------*/
/** @brief      Driver control of mecanum drive system                         */
/*-----------------------------------------------------------------------------*/

void
DriveSystemMecanumTask()
{
    int forward, right, turn;
    int temp;
    float theta;

    // Get joystick values
    DriveSystemGetControlValues( &forward, &right, &turn );

    // Field centric control
    if( theDrive.type == Mecanum4MotorFieldCentric )
    {
        // Get gyro angle in radians
        theta = degreesToRadians( GyroGetAngle() );

        // rotate coordinate system
        temp  = forward * cos(theta) - right * sin(theta);
        right = forward * sin(theta) + right * cos(theta);
        forward = temp;
    }

    // Send to drive
    DriveSystemMecanumDrive( forward, turn, right );
}

/*-----------------------------------------------------------------------------*/
/** @brief      control the arcade drive                                       */
/** @param[in]  forward The forward power/speed (-127 to 127)                  */
/** @param[in]  turn The turning power/speed (-127 to 127)                     */
/*-----------------------------------------------------------------------------*/

void
DriveSystemArcadeDrive( int forward, int turn )
{
    long drive_l_motor;
    long drive_r_motor;

    // Set drive
    drive_l_motor = forward + turn;
    drive_r_motor = forward - turn;

    // normalize drive so max is 127 if any drive is over 127
    int max = abs(drive_l_motor);
    if (abs(drive_r_motor)  > max)
        max = abs(drive_r_motor);
    if (max>127) {
        drive_l_motor = 127 * drive_l_motor / max;
        drive_r_motor = 127 * drive_r_motor  / max;
    }

    // Get control of motors
    MotorGetSemaphore();

    // Send to motors
    // left drive
    SetMotor( theDrive.leftWheels[0], drive_l_motor);
    if( theDrive.type == Arcade4Motor )
        SetMotor( theDrive.leftWheels[1], drive_l_motor);

    // right drive
    SetMotor( theDrive.rightWheels[0], drive_r_motor);
    if( theDrive.type == Arcade4Motor )
        SetMotor( theDrive.rightWheels[1], drive_r_motor);

    // release control of motors
    MotorReleaseSemaphore();
}

/*-----------------------------------------------------------------------------*/
/** @brief      Driver control of arcade drive system                          */
/*-----------------------------------------------------------------------------*/

void
DriveSystemArcadeTask()
{
    int forward, turn, tmp;

    // Get joystick values
    DriveSystemGetControlValues( &forward, &turn, &tmp );

    // Send to drive
    DriveSystemArcadeDrive( forward, turn );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Send control values to drive, selects arcade or mecanum        */
/** @param[in]  forward The forward power/speed (-127 to 127)                  */
/** @param[in]  turn The turning power/speed (-127 to 127)                     */
/** @param[in]  right The strafing power/speed (-127 to 127)                   */
/*-----------------------------------------------------------------------------*/

void
DriveSystemDrive( int forward, int turn, int right = 0 )
{
        switch( theDrive.type )
        {
        case      Mecanum4Motor:
        case      Mecanum4MotorFieldCentric:
            DriveSystemMecanumDrive( forward, turn, right );
            break;

        case      Arcade2Motor:
        case      Arcade4Motor:
            DriveSystemArcadeDrive( forward, turn );
            break;
        default:
            break;
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Task to manually control drive                                 */
/*-----------------------------------------------------------------------------*/

task
DriveSystemTask()
{
    while(1)
    {
        switch( theDrive.type )
        {
        case      Mecanum4Motor:
            DriveSystemMecanumTask();
            break;
        case      Mecanum4MotorFieldCentric:
            DriveSystemMecanumTask();
            break;
        case      Arcade2Motor:
            DriveSystemArcadeTask();
            break;
        case      Arcade4Motor:
            DriveSystemArcadeTask();
            break;
        default:
            break;
        }

        wait1Msec(25);
    }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Task to keep drive on correct heading during autonomous        */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This task uses the requested values of forward and strafing movement to keep
 *  the robot driving in the correct direction during autonomous, it worls but
 *  is expermiental.
 */

task
DriveSystemGyroTask()
{
    float gyro_angle;
    int   turn;
    float gyro_error;

    while(1)
        {
        turn = theDrive.turn_req;

        if( theDrive.turn_req == 0 )
            {
            gyro_angle = GyroGetAbsAngle();
            gyro_error = gyro_angle - theDrive.keep_angle;

            // 1 degree tolerance
            if( abs( gyro_error ) > 1.0 )
                {
                if( theDrive.forward_req != 0 )
                    turn = sgn( gyro_error) * abs(theDrive.forward_req) * 3 / 10;
                else
                if( theDrive.right_req != 0 )
                    turn = sgn( gyro_error) * abs(theDrive.right_req) * 3 / 10;
                }
            }
        else
            {
            theDrive.keep_angle = GyroGetAbsAngle();
            }

        // send to drive
        DriveSystemDrive( theDrive.forward_req, turn, theDrive.right_req );

        wait1Msec(10);
        }
}


/*-----------------------------------------------------------------------------*/
/** @brief      Send control values to drive                                   */
/** @param[in]  forward The forward power/speed (-127 to 127)                  */
/** @param[in]  turn The turning power/speed (-127 to 127)                     */
/** @param[in]  right The strafing power/speed (-127 to 127)                   */
/*-----------------------------------------------------------------------------*/
/** @details
 *  USe this instead of DriveSystemDrive to use the gyro to stabilise movement
 */

void
DriveSystemGyroDrive( int forward, int turn, int right = 0 )
{
    // Store current values
    theDrive.forward_req = forward;
    theDrive.turn_req    = turn;
    theDrive.right_req   = right;

    // Start task if it's not running
    // should only be used in autonomous
    if( getTaskState( DriveSystemGyroTask ) == taskStateStopped )
        StartTask( DriveSystemGyroTask );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Disable gyro driving                                           */
/*-----------------------------------------------------------------------------*/

void
DriveSystemGyroDriveDisable()
{
    // Stop gyro drive task if is running
    if( getTaskState( DriveSystemGyroTask ) != taskStateStopped )
        StopTask( DriveSystemGyroTask );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Save current gyro angle as the drive orientation               */
/*-----------------------------------------------------------------------------*/

void
DriveSystemGyroSetAngle()
{
    theDrive.keep_angle = GyroGetAbsAngle();
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get the saved drive orientation                                */
/** @returns    the saved drive angle                                          */
/*-----------------------------------------------------------------------------*/

float
DriveSystemGyroGetAngle()
{
    return( theDrive.keep_angle );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Start the drive system                                         */
/*-----------------------------------------------------------------------------*/

void
DriveSystemRun()
{
#ifdef _Target_Emulator_
    StartTask( DriveSystemTask );
#else
    if( bIfiAutonomousMode == false )
        StartTask( DriveSystemTask );
#endif
}

/*-----------------------------------------------------------------------------*/
/** @brief      Stop the drive system                                          */
/*-----------------------------------------------------------------------------*/

void
DriveSystemStop()
{
    int     i;
    StopTask( DriveSystemTask );

     // Get control of motors
    MotorGetSemaphore();

    // Stop all motors
    for(i=0;i<theDrive.wheelCount;i++)
        {
        SetMotor( theDrive.leftWheels[i],  0);
        SetMotor( theDrive.rightWheels[i], 0);
        }

    // release control of motors
    MotorReleaseSemaphore();
}

#endif // __DRIVELIB__
