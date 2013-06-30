/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2012                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     OpenSourceBot_Intake.c                                       */
/*    Author:     James Pearman                                                */
/*    Created:    24 Oct 2012                                                  */
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
/** @file    OpenSourceBot_Intake.c
  * @brief   Intake control on the open source robot
*//*---------------------------------------------------------------------------*/

#ifndef __INTAKELIB__
#define __INTAKELIB__

#ifndef __CTLLIB__
#include "Libraries\CtlLib.c"
#endif

#define kMaxIntakeMotors                4
#define kMaxIntakePresets              10
#define kMaxIntakeControlChannelCount   4

typedef struct {
    tSensors        sensor_port;
    tSensors        index_port;

    pidController   *intake_pid;

    // used during autonomous
    short           auton_intake_pos;

    // Up to 4 motors for intake
    short           motor_num;
    tMotor          motors[kMaxIntakeMotors];

    // offset to auto position
    short           manual_offset;
    short           preset_offset;

    // used to reset encoder in a synchronous way
    short           sensor_reset;

    // controllers
    vexrtController controller;

    // presets
    vexPreset       presets;

    float           armPositionDegrees;

    } intakeController;

static  intakeController theIntake;

void    IntakeSystemRun();
void    IntakeSystemStop();

/*-----------------------------------------------------------------------------*/
/** @brief      Intake system init                                             */
/** @param[in]  m1 Motor 1                                                     */
/** @param[in]  m2 Motor 2 (optional)                                          */
/** @param[in]  m3 Motor 3 (optional)                                          */
/** @param[in]  m4 Motor 4 (optional)                                          */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Initialize the intake system, upto 4 motors can be assigned
 */

void
IntakeSystemInit(tSensors position_port, tSensors index_port, short m1, short m2 = (-1), short m3 = (-1), short m4 = (-1))
{
    // Init - no bias
    theIntake.intake_pid = PidControllerInit( 0.035, 0.0, 0.05, position_port, 0 );
    theIntake.intake_pid->error_threshold = 5;

    // meaningless unless it is an encoder
    SensorValue[position_port] = 0;

    // save encoder port
    theIntake.sensor_port = position_port;
    // save encoder index port
    theIntake.index_port  = index_port;

    // 4 motors max
    theIntake.motors[0] = (tMotor) m1;
    theIntake.motors[1] = (tMotor) m2;
    theIntake.motors[2] = (tMotor) m3;
    theIntake.motors[3] = (tMotor) m4;

    // figure out how many for later
    theIntake.motor_num = 1;
    if( m2 != (-1) )
        theIntake.motor_num++;
    if( m3 != (-1) )
        theIntake.motor_num++;
    if( m4 != (-1) )
        theIntake.motor_num++;

    // set slew rate faster for this intake
    if( m1 != (-1) )
        SmartMotorSetSlewRate(m1, 100);
    if( m2 != (-1) )
        SmartMotorSetSlewRate(m2, 100);
    if( m3 != (-1) )
        SmartMotorSetSlewRate(m3, 100);
    if( m4 != (-1) )
        SmartMotorSetSlewRate(m4, 100);

    // Init presets
    PresetInit( &theIntake.presets );
    PresetSetTolerance( &theIntake.presets, 1 );

    // Init controller
    ControllerInit( &theIntake.controller );

    // no arm position
    theIntake.armPositionDegrees = 0.0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Test intake system indexing                                    */
/*-----------------------------------------------------------------------------*/

void
IntakeSystemIndexTest()
{
    int     i;
    long    timeout;
    int     sensor_value;
    int     old_sensor_value;

    // Disable any tasks
    IntakeSystemStop();

    old_sensor_value = SensorValue[ theIntake.index_port ];

    // move forward slowly
    for(i=0;i<theIntake.motor_num;i++)
        SetMotor( theIntake.motors[ i ], 64 );

    for(timeout=nSysTime;timeout>(nSysTime-10000);)
        {
        sensor_value = SensorValue[ theIntake.index_port ];
        if( sensor_value != old_sensor_value )
            {
            writeDebugStreamLine("change %d %d", sensor_value,SensorValue[ theIntake.sensor_port] );

            }
        old_sensor_value = sensor_value;
        wait1Msec(10);
        }

    // stop
    for(i=0;i<theIntake.motor_num;i++)
       SetMotor( theIntake.motors[ i ], 0 );

    // Start task
    IntakeSystemRun();
}

/*-----------------------------------------------------------------------------*/
/** @brief      Intake system calibrate                                        */
/*-----------------------------------------------------------------------------*/

void
IntakeSystemCalibrate()
{
    int     i;
    long    timeout;

    // Disable any tasks
    IntakeSystemStop();

    // index should be closed at start
    if( SensorValue[ theIntake.index_port ] == 0 )
        {
        // move forward slowly
        for(i=0;i<theIntake.motor_num;i++)
            SetMotor( theIntake.motors[ i ], 32 );

        for(timeout=nSysTime;timeout>(nSysTime-2000);)
            {
            // Look for index open
            if( SensorValue[ theIntake.index_port ] == 1 )
                {
                break;
                }

            wait1Msec(10);
            }

        // stop
        for(i=0;i<theIntake.motor_num;i++)
            SetMotor( theIntake.motors[ i ], 0 );

        // reset encoder
        SensorValue[ theIntake.sensor_port ] = 0;

        // 1/2 sec delay
        wait1Msec(500);
        writeDebugStreamLine("Trigger at %d", SensorValue[ theIntake.sensor_port ] );

        // move forward slowly
        for(i=0;i<theIntake.motor_num;i++)
            SetMotor( theIntake.motors[ i ], 32 );

        // move until home position
        for(timeout=nSysTime;timeout>(nSysTime-2000);)
            {
            // Look for index open
            if( SensorValue[ theIntake.sensor_port ] >= 50 )
                {
                break;
                }

            wait1Msec(10);
            }

        // stop
        for(i=0;i<theIntake.motor_num;i++)
            SetMotor( theIntake.motors[ i ], 0 );

        wait1Msec(500);
        writeDebugStreamLine("Stop at %d", SensorValue[ theIntake.sensor_port ] );

        // reset encoder
        SensorValue[ theIntake.sensor_port ] = 0;
        }

    // Start task
    IntakeSystemRun();
}

/*-----------------------------------------------------------------------------*/
/** @brief      Add a preset to the list of intake offset positions            */
/** @param[in]  offset an offset to store as a preset                          */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Works differently to the arm presets as these are offsets rather than
 *  absolute positions
 */

void
IntakeSystemAddPresetOffset( int offset )
{
    PresetAdd( &theIntake.presets, offset );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Attach controls to the Intake                                  */
/** @param[in]  c1 A control index (Ch1, Btn8U etc)                            */
/** @param[in]  c2 A control index (optional)                                  */
/** @param[in]  c3 A control index (optional)                                  */
/** @param[in]  c4 A control index (optional)                                  */
/*-----------------------------------------------------------------------------*/

void
IntakeSystemAttachControls( short c1, short c2 = (-1), short c3 = (-1), short c4 = (-1))
{
    ControllerAttachControls( &theIntake.controller, c1, c2, c3, c4 );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Attach enable button to the Intake                             */
/** @param[in]  c1 A control index (Ch1, Btn8U etc)                            */
/** @param[in]  state enabled state (0 or 1)                                   */
/** @param[in]  channel_mask A mask that determines which channels are disabled*/
/*-----------------------------------------------------------------------------*/

void
IntakeSystemAttachEnableButton( short c1, short state, short channel_mask = 0xFF )
{
    ControllerAttachEnableButton( &theIntake.controller, c1, state, channel_mask );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Set the Intake controller channel scaling                      */
/** @param[in]  channel The control channel                                    */
/** @param[in]  scale The scale factor (0.0 to 1.0)                            */
/*-----------------------------------------------------------------------------*/
/** @details
 *  change the scaling if we are using buttons for control
 *  for example, if we want +/- 64 to be maximum control values then set scale
 *  as 0.5
 */

void
IntakeSystemSetChannelScale( int channel, float scale )
{
    if( (scale < 0.0) || (scale > 1.0))
        return;

    if( channel == 0 ) {
        ControllerSetChannelScale( &theIntake.controller, CTL00_P, scale );
        ControllerSetChannelScale( &theIntake.controller, CTL00_N, scale );
        }
    if( channel == 1 ) {
        ControllerSetChannelScale( &theIntake.controller, CTL01_P, scale );
        ControllerSetChannelScale( &theIntake.controller, CTL01_N, scale );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get the intake control values                                  */
/** @param[in]  ctl0 Pointer to storage for first control variable             */
/** @param[in]  ctl1 Pointer to storage for second control variable            */
/*-----------------------------------------------------------------------------*/

void
IntakeSystemGetControlValues( int *ctl0, int *ctl1 )
{
    ControllerGetControlValues( &theIntake.controller, ctl0, ctl1 );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get a preset position                                          */
/** @param[in]  intake Pointer to intake control structure                     */
/** @param[in]  dir forwards or backwards                                      */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Based on the current target positon determine the next preset position
 */

int
IntakeSystemGetPreset( intakeController *intake, short dir )
{
    return( PresetGetNextPosition( &intake->presets, intake->preset_offset, dir ) );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Control the intake                                             */
/** @param[in]  position an optional target position                           */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Normally the intake is locked to the arm position
 *  send a position to override that calculation
 */

void
ArmSystemDoIntakeControl( short position = (-1) )
{
    int     i;
    int     intake_pos;

    // if no manual position then calculate auto position
    if( position == (-1) )
        {
        // auto
        // no scaling as encoder is in degrees already
        intake_pos = theIntake.armPositionDegrees;
        }
    else
        // manual position overrides auto gearing to arm
        intake_pos = position;

    // drive intake - add offset
    theIntake.intake_pid->target_value = intake_pos + theIntake.manual_offset + theIntake.preset_offset;

    // Intake PID
    PidControllerUpdate( theIntake.intake_pid );

    // Kill if power is lost
    if( nImmediateBatteryLevel < 3000 )
        theIntake.intake_pid->drive_cmd = 0;

   // send to motors
    for(i=0;i<theIntake.motor_num;i++)
        SetMotor( theIntake.motors[ i ], theIntake.intake_pid->drive_cmd);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Task to control intake during driver control period            */
/*-----------------------------------------------------------------------------*/

task IntakeSystemTask()
{
    int manual_control, preset_control;
    int preset_action = 0;

    while(1)
        {
        // Get joystick values
        IntakeSystemGetControlValues( &manual_control, &preset_control );

        // Clip preset control (1, 0 or -1)
        preset_control = sgn( preset_control );

        // Next preset
        if( ( preset_control > 0 ) && (!preset_action) )
            {
            // detect button push on first loop
            preset_action = 1;
            // next preset
            if( IntakeSystemGetPreset( &theIntake, 1 ) != (-1) )
                theIntake.preset_offset = PresetCurrentPosition( &theIntake.presets );
            }
        else
        // Previous preset
        if( ( preset_control < 0 ) && (!preset_action) )
            {
            // detect button push on first loop
            preset_action = 1;
            // previous preset
            if( IntakeSystemGetPreset( &theIntake, 0 ) != (-1) )
                theIntake.preset_offset = PresetCurrentPosition( &theIntake.presets );
            }
        else
            {
            // wait for button release
            if( preset_control == 0 )
                {
                // use manual
                preset_action = 0;
                theIntake.manual_offset = manual_control;
                }
            }

        theIntake.armPositionDegrees = ArmSystemGetPositionDegrees();

        // Now control the intake
        ArmSystemDoIntakeControl();

        // don't hog CPU
        wait1Msec( 10 );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Task to control intake during autonomous control period        */
/*-----------------------------------------------------------------------------*/

task IntakeSystemAutonTask()
{
    while(1)
        {
        theIntake.armPositionDegrees = ArmSystemGetPositionDegrees();

        ArmSystemDoIntakeControl(theIntake.auton_intake_pos);

        // don't hog CPU
        wait1Msec( 25 );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Start the intake system                                        */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Start task for intake control based on the auntonomous system flag
 *  Force manual if running on the emulator
 */

void
IntakeSystemRun()
{
#ifdef _Target_Emulator_
    StartTask( IntakeSystemTask );
    StopTask( IntakeSystemAutonTask );
#else
    if( bIfiAutonomousMode == true )
        {
        StopTask( IntakeSystemTask );
        StartTask( IntakeSystemAutonTask );
        }
    else
        {
        StopTask( IntakeSystemAutonTask );
        StartTask( IntakeSystemTask );
        }
#endif
}

/*-----------------------------------------------------------------------------*/
/** @brief      Stop the intake system                                         */
/*-----------------------------------------------------------------------------*/

void
IntakeSystemStop()
{
    int     i;

    StopTask( IntakeSystemTask );
    StopTask( IntakeSystemAutonTask );

    // Stop motors
    for(i=0;i<theIntake.motor_num;i++)
        SetMotor( theIntake.motors[ i ], 0);
}


#endif  // __INTAKELIB__
