/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     OpenSourceBot_lcd2.c                                         */
/*    Author:     James Pearman                                                */
/*    Created:    31 March 2013                                                */
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
/*    LCD display for the full version of the open source robot code           */
/*-----------------------------------------------------------------------------*/


typedef enum {
    kLcdDispStart = 0,

    kLcdDispDriveSpeed = 0,
    kLcdDispDriveCurrent,
    kLcdDispDriveTemp,
    kLcdDispDriveMotors,
    kLcdDispDrivePosition,

    kLcdDispArmSpeed,
    kLcdDispArmCurrent,
    kLcdDispArmPosition,

    kLcdDispSonar,
    kLcdDispLineFollow,

    kLcdDispI2CStatus,
    kLcdDispSysStatus,

    kLcdDispGyro,
    kLcdDispCalibrate,

    kLcdDispNumber
    } kLcdDispType;

kLcdDispType    mode = kLcdDispSysStatus;

#if kRobotCVersionNumeric >= 360
static TI2cStatistics i2cstat;
#endif

// forward ref
void    IntakeSystemUnfold(void);

/*-----------------------------------------------------------------------------*/
/*  Utility functions to display various robot variables                       */
/*-----------------------------------------------------------------------------*/

void
LcdDisplayMotors( tMotor m1, tMotor m2, tMotor m3 = (-1), tMotor m4 = (-1) )
{
    string str;

    sprintf( str, "%4d  %4d    ", motor[ m1 ], motor[ m2 ]);
    displayLCDString(0, 0, str);
    if(m3!=(-1)) {
        sprintf( str, "%4d  %4d    ", motor[ m3 ], motor[ m4 ]);
        displayLCDString(1, 0, str);
    }
}

/*-----------------------------------------------------------------------------*/

void
LcdDisplayMotorEncoders( tMotor m1, tMotor m2 = (-1), tMotor m3 = (-1), tMotor m4 = (-1) )
{
    string str;

    if(m2==(-1))
        sprintf( str, "%7d         ",nMotorEncoder[ m1 ]);
    else
        sprintf( str, "%7d  %7d",nMotorEncoder[ m1 ], nMotorEncoder[ m2 ]);

    displayLCDString(0, 0, str);

    if(m3!=(-1)) {
        if(m4==(-1))
            sprintf( str, "%7d         ",nMotorEncoder[ m3 ]);
        else
            sprintf( str, "%7d  %7d",nMotorEncoder[ m3 ], nMotorEncoder[ m4 ]);

        displayLCDString(1, 0, str);
    }
}

/*-----------------------------------------------------------------------------*/

void
LcdDisplaySpeeds( tMotor m1, tMotor m2, tMotor m3 = (-1), tMotor m4 = (-1) )
{
    string str;

    sprintf( str, "%7.2f  %7.2f", SmartMotorGetSpeed( m1 ), SmartMotorGetSpeed( m2 ));
    displayLCDString(0, 0, str);
    if(m3!=(-1)) {
        sprintf( str, "%7.2f  %7.2f", SmartMotorGetSpeed( m3 ), SmartMotorGetSpeed( m4 ));
        displayLCDString(1, 0, str);
    }
}

/*-----------------------------------------------------------------------------*/

void
LcdDisplayCurrent( tMotor m1, tMotor m2, tMotor m3 = (-1), tMotor m4 = (-1))
{
    string str;

    sprintf( str, "%7.2f  %7.2f", SmartMotorGetCurrent( m1 ), SmartMotorGetCurrent( m2 ));
    displayLCDString(0, 0, str);
    if(m3!=(-1)) {
        sprintf( str, "%7.2f  %7.2f", SmartMotorGetCurrent( m3 ), SmartMotorGetCurrent( m4 ));
        displayLCDString(1, 0, str);
    }
}

/*-----------------------------------------------------------------------------*/

void
LcdDisplayTemperature( tMotor m1, tMotor m2, tMotor m3, tMotor m4 )
{
    string str;

    sprintf( str, "%7.2f  %7.2f", SmartMotorGetTemperature( m1 ), SmartMotorGetTemperature( m2 ));
    displayLCDString(0, 0, str);
    sprintf( str, "%7.2f  %7.2f", SmartMotorGetTemperature( m3 ), SmartMotorGetTemperature( m4 ));
    displayLCDString(1, 0, str);
}

/*-----------------------------------------------------------------------------*/

void
LcdDisplaySysStatus(  )
{
    string str;

    sprintf(str,"VBatt %7.2f   ", nAvgBatteryLevel/1000.0 );
    displayLCDString(0, 0, str);
    //sprintf(str,"%s %s", kRobotCVersion, pzFirmwareVersion );
    displayLCDString(1, 0, kRobotCVersion);
    displayLCDString(1, 6, pzFirmwareVersion);
}

/*-----------------------------------------------------------------------------*/

void
LcdDisplayLineFollowStatus( tSensors s1, tSensors s2, tSensors s3 )
{
    string str;

    displayLCDString(0, 0, "Line Sensors    ");
    sprintf( str, "%4d %4d %4d ", SensorValue[ s1 ], SensorValue[ s2 ], SensorValue[ s3 ] );
    displayLCDString(1, 0, str);
}

/*-----------------------------------------------------------------------------*/

void
LcdDisplaySonarStatus( tSensors s1 )
{
    string str;

    displayLCDString(0, 0, "Rear Sonar      ");
    sprintf( str, "%4d", SensorValue[ s1 ] );
    displayLCDString(1, 0, str);
}

/*-----------------------------------------------------------------------------*/
/*  Call this function to display robot variables and status                   */
/*  Use the LCD left and right buttons to change the displayed information     */
/*  You can also use joystick buttons 7D and 7R for the same function          */
/*-----------------------------------------------------------------------------*/

void
LcdDisplayStatus()
{
    string str;
    TControllerButtons    Buttons;

    // Select display Item
    Buttons = nLCDButtons;
    if( vexRT[ Btn7D ] )
        Buttons = kButtonLeft;
    if( vexRT[ Btn7R ] )
        Buttons = kButtonRight;

    if( (Buttons == kButtonLeft) || (Buttons == kButtonRight) )
        {
        if( Buttons == kButtonRight )
           {
            mode++;
            if(mode >= kLcdDispNumber)
                mode = kLcdDispStart;
            }
        if( Buttons == kButtonLeft )
            {
            mode--;
            if(mode < kLcdDispStart)
                mode = (kLcdDispNumber-1);
            }

        clearLCDLine(0);
        clearLCDLine(1);

        switch(mode)
            {
            case    kLcdDispDriveSpeed:
                displayLCDString(0,0, "Speed Drive     ");
                break;
            case    kLcdDispDriveCurrent:
                displayLCDString(0,0, "Current Drive   ");
                break;
            case    kLcdDispDriveTemp:
                displayLCDString(0,0, "Temp Drive      ");
                break;
            case    kLcdDispDriveMotors:
                displayLCDString(0,0, "Motor Drive     ");
                break;
            case    kLcdDispDrivePosition:
                displayLCDString(0,0, "Position Drive  ");
                break;

            case    kLcdDispArmSpeed:
                displayLCDString(0,0, "Speed Arm/Intake");
                break;
            case    kLcdDispArmCurrent:
                displayLCDString(0,0, "Current Arm/Intk");
                break;
            case    kLcdDispArmPosition:
                displayLCDString(0,0, "Position Arm    ");
                break;

            case    kLcdDispSonar:
                displayLCDString(0,0, "Rear Sonar      ");
                break;
            case    kLcdDispLineFollow:
                displayLCDString(0,0, "Line Sensors    ");
                break;
            case    kLcdDispI2CStatus:
                displayLCDString(0,0, "I2C Status      ");
                break;
            case    kLcdDispSysStatus:
                displayLCDString(0,0, "Status          ");
                break;

            case    kLcdDispGyro:
                displayLCDString(0,0, "Gyro            ");
                break;
            case    kLcdDispCalibrate:
                displayLCDString(0,0, "Calibrate intake");
                break;

            default:
                displayLCDString(0,0, "Err             ");
                break;
            }

        do {
            Buttons = nLCDButtons;
            if( vexRT[ Btn7D ] )
                Buttons = kButtonLeft;
            if( vexRT[ Btn7R ] )
                Buttons = kButtonRight;
            wait1Msec(10);
            } while( Buttons != kButtonNone );


        wait1Msec(250);
        }

   switch( mode )
        {
        case    kLcdDispDriveSpeed:
            LcdDisplaySpeeds( MotorLF, MotorRF, MotorLB, MotorRB );
            break;

        case    kLcdDispDriveCurrent:
            LcdDisplayCurrent( MotorLF, MotorRF, MotorLB, MotorRB );
            break;

        case    kLcdDispDriveTemp:
            LcdDisplayTemperature( MotorLF, MotorRF, MotorLB, MotorRB );
            break;

        case    kLcdDispDriveMotors:
            LcdDisplayMotors( MotorLF, MotorRF, MotorLB, MotorRB );
            break;

        case    kLcdDispDrivePosition:
            LcdDisplayMotorEncoders( MotorLF, MotorRF, MotorLB, MotorRB );
            break;


        case    kLcdDispArmSpeed:
            LcdDisplaySpeeds( MotorArmL, MotorArmR, MotorIL, MotorIR );
            break;

        case    kLcdDispArmCurrent:
            LcdDisplayCurrent( MotorArmL, MotorArmR, MotorIL, MotorIR);
            break;

        case    kLcdDispArmPosition:
            LcdDisplayMotorEncoders( MotorArmR );
            break;

        case    kLcdDispSonar:
            LcdDisplaySonarStatus( RearSonar );
            break;

        case    kLcdDispLineFollow:
            LcdDisplayLineFollowStatus( LineFollowC, LineFollowB, LineFollowA );
            break;

        case    kLcdDispI2CStatus:
#if kRobotCVersionNumeric >= 360
            sprintf( str, "RST %2d ERR %d", i2cstat.nTotalAddressResets, i2cstat.bI2CNeverResponded );
            displayLCDString(1, 0, str);
#endif
            break;

        case    kLcdDispSysStatus:
            LcdDisplaySysStatus();
            break;

        case    kLcdDispGyro:
            GyroDebug(0);
            break;

        case    kLcdDispCalibrate:
            displayLCDString(0,0,"Calibrate intake ");
            break;

       default:
            displayLCDString(0, 0, "error" );
            break;
        }

    // Reinit Gyro
    if( mode == kLcdDispGyro && nLCDButtons == kButtonCenter )
        {
        // Wait for release
        while( nLCDButtons != kButtonNone )
            wait1Msec( 25 );
        // Kill and reinit Gyro
        GyroReinit();

        clearLCDLine(0);
        }

   // Calibrate intake system
   if( mode == kLcdDispCalibrate && nLCDButtons == kButtonCenter )
        {
        // Wait for release
        while( nLCDButtons != kButtonNone )
            wait1Msec( 25 );

        // recal intake
        IntakeSystemUnfold();
        displayLCDString(0,0,"Done             ");
        }
}
