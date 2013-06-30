/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2012                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     OpenSourceBot_Arm.c                                          */
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
/** @file    OpenSourceBot_Arm.c
  * @brief   Arm control on the open source robot
*//*---------------------------------------------------------------------------*/

#ifndef __ARMLIB__
#define __ARMLIB__

#ifndef __CTLLIB__
#include "Libraries\CtlLib.c"
#endif

/** @brief  Maximum number of motor we can handle on the arm
 */
#define kMaxArmMotors       4

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  All the variables for arm control                                          */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

typedef struct {
    tSensors        sensor_port;

    long            upper_limit;
    long            lower_limit;

    pidController   *arm_pid;

    // Up to 4 motors for arm
    short           motor_num;
    tMotor          motors[kMaxArmMotors];

    // override power to arm in case PTC trips
    short           motor_off;

    // manual overide of PID control
    short           user_manual;

    // controller
    vexrtController controller;

    // presets
    vexPreset       presets;

    tSensors        limitSwitchLow;
    tSensors        limitSwitchHigh;

    // sensor counts per degree after gearing
    float           CountsPerDeg;
    } armController;

static  armController   theArm;

/*-----------------------------------------------------------------------------*/
/** @brief      Arm system init                                                */
/** @param[in]  m1 Motor 1                                                     */
/** @param[in]  m2 Motor 2 (optional)                                          */
/** @param[in]  m3 Motor 3 (optional)                                          */
/** @param[in]  m4 Motor 4 (optional)                                          */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Initialize the arm system, upto 4 motors can be assigned
 */

void
ArmSystemInit( tSensors position_port, short m1, short m2 = (-1), short m3 = (-1), short m4 = (-1) )
{
    // Init - no bias
    theArm.arm_pid = PidControllerInit( 0.004, 0.0, 0.01, position_port, 0 );

    // meaningless unless it is an encoder
    SensorValue[position_port] = 0;

    // save port
    theArm.sensor_port = position_port;

    // 4 motors max
    theArm.motors[0] = (tMotor) m1;
    theArm.motors[1] = (tMotor) m2;
    theArm.motors[2] = (tMotor) m3;
    theArm.motors[3] = (tMotor) m4;

    // figure out how many for later
    theArm.motor_num = 1;
    if( m2 != (-1) )
        theArm.motor_num++;
    if( m3 != (-1) )
        theArm.motor_num++;
    if( m4 != (-1) )
        theArm.motor_num++;

    // Init presets
    PresetInit( &theArm.presets );

    // Init controller
    ControllerInit( &theArm.controller );
    // default control
    ControllerAttachControls( &theArm.controller, Ch2, Btn8U, Btn8D );

    // default upper and lower limits
    theArm.upper_limit = 2000;
    theArm.lower_limit = 500;

    theArm.limitSwitchLow  = (tSensors)(-1);
    theArm.limitSwitchHigh = (tSensors)(-1);

    // normal control
    theArm.user_manual = 0;

    // encoder counts per degree at the arm
    // composite gearing for this arm
    theArm.CountsPerDeg = 627.2 * (3600/432) / 360;
}


/*-----------------------------------------------------------------------------*/
/** @brief      Set upper and lower position limits                            */
/** @param[in]  lower The lower limit sensor value                             */
/** @param[in]  upper The upper limit sensor value                             */
/*-----------------------------------------------------------------------------*/

void
ArmSystemSetLimits( long lower, long upper )
{
    theArm.upper_limit = upper;
    theArm.lower_limit = lower;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Set upper and lower limit switch sensors                       */
/** @param[in]  low The lower limit switch                                     */
/** @param[in]  high The upper limit switch                                    */
/*-----------------------------------------------------------------------------*/

void
ArmSystemSetLimitSwitches( tSensors low, tSensors high = (-1) )
{
    theArm.limitSwitchLow  = low;
    theArm.limitSwitchHigh = high;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Add a preset to the list of Arm preset positions               */
/** @param[in]  position The preset position                                   */
/*-----------------------------------------------------------------------------*/
/** @Note       Presets should be added in incrementing order in this version
 *              may add sort later.
 */

void
ArmSystemAddPresetPositions( long position )
{
    PresetAdd( &theArm.presets, position );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Disable arm motors                                             */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Overide the PID loop on the arm motors
 */

void
ArmSystemDisableMotors()
{
    theArm.motor_off = 1;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Enable arm motors                                              */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Allow PID control of the arm motors
 */

void
ArmSystemEnableMotors()
{
    theArm.motor_off = 0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Attach controls to the arm                                     */
/** @param[in]  c1 A control index (Ch1, Btn8U etc)                            */
/** @param[in]  c2 A control index (optional)                                  */
/** @param[in]  c3 A control index (optional)                                  */
/** @param[in]  c4 A control index (optional)                                  */
/** @param[in]  c5 A control index (optional)                                  */
/** @param[in]  c6 A control index (optional)                                  */
/*-----------------------------------------------------------------------------*/

void
ArmSystemAttachControls( short c1, short c2 = (-1), short c3 = (-1), short c4 = (-1), short c5 = (-1), short c6 = (-1))
{
    ControllerAttachControls( &theArm.controller, c1, c2, c3, c4, c5, c6);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Attach enable button to the arm                                */
/** @param[in]  c1 A control index (Ch1, Btn8U etc)                            */
/** @param[in]  state enabled state (0 or 1)                                   */
/** @param[in]  channel_mask A mask that determines which channels are disabled*/
/*-----------------------------------------------------------------------------*/

void
ArmSystemAttachEnableButton( short c1, short state, short channel_mask = 0xFF )
{
    ControllerAttachEnableButton( &theArm.controller, c1, state, channel_mask );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Set the arm controller channel scaling                         */
/** @param[in]  channel The control channel                                    */
/** @param[in]  scale The scale factor (0.0 to 1.0)                            */
/*-----------------------------------------------------------------------------*/
/** @details
 *  change the scaling if we are using buttons for control
 *  for example, if we want +/- 64 to be maximum control values then set scale
 *  as 0.5
 */

void
ArmSystemSetChannelScale( int channel, float scale )
{
    if( (scale < 0.0) || (scale > 1.0))
        return;

    if( channel == 0 ) {
        ControllerSetChannelScale( &theArm.controller, CTL00_P, scale );
        ControllerSetChannelScale( &theArm.controller, CTL00_N, scale );
        }
    if( channel == 1 ) {
        ControllerSetChannelScale( &theArm.controller, CTL01_P, scale );
        ControllerSetChannelScale( &theArm.controller, CTL01_N, scale );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get the arm control values                                     */
/** @param[in]  ctl0 Pointer to storage for first control variable             */
/** @param[in]  ctl1 Pointer to storage for second control variable            */
/*-----------------------------------------------------------------------------*/

void
ArmSystemGetControlValues( int *ctl0, int *ctl1 )
{
    ControllerGetControlValues( &theArm.controller, ctl0, ctl1 );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get a preset position                                          */
/** @param[in]  arm Pointer to arm control structure                           */
/** @param[in]  dir forwards or backwards                                      */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Based on the current target positon determine the next preset position
 */

int
ArmSystemGetPreset( armController *arm, short dir )
{
    return( PresetGetNextPosition( &arm->presets, arm->arm_pid->target_value, dir ) );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Check the hard limit switches and kill power if triggered      */
/*-----------------------------------------------------------------------------*/

void
ArmSystemCheckHardLimits()
{
    static  int hardStopL = 0;
    static  int hardStopH = 0;
            int i;

    if( theArm.limitSwitchLow != (-1) )
        {
        if( SensorValue[ theArm.limitSwitchLow ] == 0 )
            {
            if( theArm.arm_pid->drive_cmd < 0 )
                {
                theArm.arm_pid->drive_cmd = 0;
                // hard stop
                if( !hardStopL )
                    {
                    hardStopL = 1;
                    // hard stop
                    for(i=0;i<theArm.motor_num;i++)
                        SetMotor( theArm.motors[ i ], 20, true);
                    wait1Msec(100);
                    for(i=0;i<theArm.motor_num;i++)
                        SetMotor( theArm.motors[ i ], 0, true);
                    }
                }
            }
        else
            hardStopL = 0;
        }


    if( theArm.limitSwitchHigh != (-1) )
        {
        if( SensorValue[ theArm.limitSwitchHigh ] == 0 )
            {
            if( theArm.arm_pid->drive_cmd > 0 )
                {
                theArm.arm_pid->drive_cmd = 0;
                if( !hardStopH )
                    {
                    hardStopH = 1;
                    // hard stop
                    for(i=0;i<theArm.motor_num;i++)
                        SetMotor( theArm.motors[ i ], -20, true);
                    wait1Msec(100);
                    for(i=0;i<theArm.motor_num;i++)
                        SetMotor( theArm.motors[ i ], 0, true);
                    }
                }
            }
        else
            hardStopH = 0;
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Control the arm                                                */
/** @param[in]  position an optional target position                           */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Arm control function, call from a task running every 25mS
 *  use the position parameter to override any target_value
 */

void
ArmSystemDoArmControl( short position = (-1) )
{
    int    i;

    if( position != (-1) )
        theArm.arm_pid->target_value = position;

    //if( !vexRT[ Btn7U ] && !vexRT[ Btn7L ] )
    if( !theArm.user_manual )
        {
        // clip target value to limits
        if( theArm.arm_pid->target_value > theArm.upper_limit )
            theArm.arm_pid->target_value = theArm.upper_limit;
        if( theArm.arm_pid->target_value < theArm.lower_limit )
            theArm.arm_pid->target_value = theArm.lower_limit;

        // pid control
        PidControllerUpdate( theArm.arm_pid );
        }
    else
        {
        int manual_control;

        // total manual override for emergency
        theArm.arm_pid->target_value = theArm.arm_pid->sensor_value;
        // get value of manual control again
        ControllerGetControlValues( &theArm.controller, &manual_control );
        // no PID
        theArm.arm_pid->drive_cmd = manual_control;
        }

    // overide for arm motors
    if( theArm.motor_off )
        theArm.arm_pid->drive_cmd = 0;

    // Kill if power is lost
    if( nImmediateBatteryLevel < 3000 )
        theArm.arm_pid->drive_cmd = 0;

    // check limit switches
    ArmSystemCheckHardLimits();

    // send to motors
    for(i=0;i<theArm.motor_num;i++)
        SetMotor( theArm.motors[ i ], theArm.arm_pid->drive_cmd);
}



/*-----------------------------------------------------------------------------*/
/** @brief      Set arm position during autonomous                             */
/** @param[in]  arm_preset The preset number to move to                        */
/** @param[in]  manual An absolute posiiton to go to if not 0                  */
/*-----------------------------------------------------------------------------*/

void
ArmSystemAutonSetArmPosition( short arm_preset, short manual = 0 )
{
    if( (arm_preset >= 0) && (arm_preset < PresetGetNum( &theArm.presets ) ) )
        {
        PresetSet( &theArm.presets, arm_preset);
        theArm.arm_pid->target_value = PresetCurrentPosition( &theArm.presets );
        }
    else
        {
        if( manual > 0 )
            theArm.arm_pid->target_value = manual;
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Wait for arm to be in position                                 */
/** @param[in]  timeout The maximum time to wait in mS                         */
/** @returns    success (1) or failure(0)                                      */
/*-----------------------------------------------------------------------------*/

short
ArmSystemWaitInPosition( short timeout = 3000 )
{
    // default is 3 seconds
    short   count = timeout;

    while(count >= 0 )
        {
        // wait so that pid has calculated error
        wait1Msec(50);

        // Check the pid error for the arm
        if( (abs(theArm.arm_pid->error) < 100 ) )
            return(1);

        // decrease timeout
        count -= 50;
        }

    return(0);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get the arm position                                           */
/** @returns    The arm position in degrees                                    */
/*-----------------------------------------------------------------------------*/

float
ArmSystemGetPositionDegrees()
{
    float   deg;

    deg = (float)theArm.arm_pid->sensor_value / theArm.CountsPerDeg;

    return(deg);
}


/*-----------------------------------------------------------------------------*/
/** @brief      Task to control arm during driver control period               */
/*-----------------------------------------------------------------------------*/

task ArmSystemTask()
{
    int manual_control, preset_control;
    int preset_action = 0;

    while(1)
        {
        // Get joystick values
        ArmSystemGetControlValues( &manual_control, &preset_control );
#ifdef _Target_Emulator_
        preset_control = (nLCDButtons == 1) ? 1 : (nLCDButtons == 2 ) ? (-1) : 0;
#endif
        // Clip preset control (1, 0 or -1)
        preset_control = sgn( preset_control );

        // user override, hold two buttons
        if( ControllerGetControlState( &theArm.controller, CHANNEL2 ) == 3 )
            theArm.user_manual = 1;
        else
            theArm.user_manual = 0;

        // Next preset
        if( ( preset_control > 0 ) && (!preset_action) )
            {
            // detect button push on first loop
            preset_action = 1;
            // next preset
            if( ArmSystemGetPreset( &theArm, 1 ) != (-1) )
                theArm.arm_pid->target_value = PresetCurrentPosition( &theArm.presets );
            }
        else
        // Previous preset
        if( ( preset_control < 0 ) && (!preset_action) )
            {
            // detect button push on first loop
            preset_action = 1;
            // previous preset
            if( ArmSystemGetPreset( &theArm, 0 ) != (-1) )
                theArm.arm_pid->target_value = PresetCurrentPosition( &theArm.presets );
            }
        else
            {
            // wait for button release
            if( preset_control == 0 )
                {
                // use manual
                preset_action = 0;
                theArm.arm_pid->target_value = theArm.arm_pid->target_value + (manual_control / 4);
                }
            }

        // control the arm
        ArmSystemDoArmControl();

        // don't hog CPU
        wait1Msec( 25 );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Task to control arm during autonomous control period           */
/*-----------------------------------------------------------------------------*/

task ArmSystemAutonTask()
{
    while(1)
        {
        ArmSystemDoArmControl();

        // don't hog CPU
        wait1Msec( 25 );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Start the arm system                                           */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Start task for arm control based on the autonomous system flag
 *  Force manual if running on the emulator
 */

void
ArmSystemRun()
{
#ifdef _Target_Emulator_
    StartTask( ArmSystemTask );
    StopTask( ArmSystemAutonTask );
#else
    if( bIfiAutonomousMode == true )
        {
        StopTask( ArmSystemTask );
        StartTask( ArmSystemAutonTask );
        }
    else
        {
        StopTask( ArmSystemAutonTask );
        StartTask( ArmSystemTask );
        }
#endif
}

#endif  // __ARMLIB__
