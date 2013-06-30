/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2012                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*        Module:     CtlLib.c                                                 */
/*        Author:     James Pearman                                            */
/*        Created:    28 Jan 2012                                              */
/*                                                                             */
/*    Revisions:                                                               */
/*          V1.00     30 June 2013 - Initial release                           */
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
/** @file    CtlLib.c
  * @brief   Library to abstract the vex controller
  * @details
  * Standardized controller
  * We were duplicating code in drive, arm and intake libraries so it's
  * time to pull that into its own library.
*//*---------------------------------------------------------------------------*/

// Stop recursive includes
#ifndef __CTLLIB__
#define __CTLLIB__

// Should be even
#define kControlChannelCount      8

typedef struct {
    // Control channels
    short       ctl[ kControlChannelCount ];
    // control channel scaling
    float       ctlscale[ kControlChannelCount ];
    // joystick threshold
    int         threshold;
    // Enable buttons
    short       ctlebl;
    short       ctleblstate;
    short       ctleblmask;
    } vexrtController;

// control channel definitions
#define CTL00_P      0
#define CTL01_P      2
#define CTL02_P      4
#define CTL03_P      6

#define CTL00_N      1
#define CTL01_N      3
#define CTL02_N      5
#define CTL03_N      7

// for external use
#define CHANNEL0     CTL00_P
#define CHANNEL1     CTL01_P
#define CHANNEL2     CTL02_P
#define CHANNEL3     CTL03_P

/*-----------------------------------------------------------------------------*/
/** @brief      Initialize a controller                                        */
/** @param[in]  c The controller                                               */
/*-----------------------------------------------------------------------------*/

void
ControllerInit( vexrtController *c )
{
    int     channel;

    for(channel=0;channel<kControlChannelCount;channel++)
         {
        c->ctl[channel]      = (-1);
        c->ctlscale[channel] = 1.0;
        }

    // joystick threshold
    c->threshold   = 10;

    c->ctlebl      = (-1);
    c->ctleblstate = 0;
    c->ctleblmask  = 0xFF;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Attach controls to the controller                              */
/** @param[in]  c The controller                                               */
/** @param[in]  c1 A control index (Ch1, Btn8U etc)                            */
/** @param[in]  c2 A control index (optional)                                  */
/** @param[in]  c3 A control index (optional)                                  */
/** @param[in]  c4 A control index (optional)                                  */
/** @param[in]  c5 A control index (optional)                                  */
/** @param[in]  c6 A control index (optional)                                  */
/** @param[in]  c7 A control index (optional)                                  */
/** @param[in]  c8 A control index (optional)                                  */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Control can either be with joysticks or buttons or any combinations
 *  If joystick control is used then only the "P" channel is assigned
 *  If button control is used then both P and N channels are used for positive
 *  and negative control
 *
 */

void
ControllerAttachControls( vexrtController *c, short c1, short c2 = (-1), short c3 = (-1), short c4 = (-1), short c5 = (-1), short c6 = (-1), short c7 = (-1), short c8 = (-1))
{
    short   ca[kControlChannelCount] = {c1, c2, c3, c4, c5, c6, c7, c8};
    int     cp = 0;
    int     channel;

    for(channel=0;channel<kControlChannelCount;channel+=2)
        {
        c->ctl[channel] = ca[cp++];

        if( c->ctl[channel] >= (short)Btn5D )
            c->ctl[channel+1] = ca[cp++];
        else
            c->ctl[channel+1] = (-1);
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Attach enable button to the controller                         */
/** @param[in]  c The controller                                               */
/** @param[in]  c1 A control index (Ch1, Btn8U etc)                            */
/** @param[in]  state enabled state (0 or 1)                                   */
/** @param[in]  mask A mask that determines which channels are disabled        */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Attach a control to the controller to act as an enable button
 *  This allows, for example, a joystick to control more than one thing
 */

void
ControllerAttachEnableButton( vexrtController *c, short c1, short state = 1, short mask = 0xFF )
{
    c->ctlebl = c1;
    c->ctleblstate = state;
    c->ctleblmask  = mask;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Set the controller channel scaling                             */
/** @param[in]  c The controller                                               */
/** @param[in]  channel The control channel                                    */
/** @param[in]  scale The scale factor (0.0 to 1.0)                            */
/*-----------------------------------------------------------------------------*/
/** @details
 *  change the scaling if we are using buttons for control
 *  for example, if we want +/- 64 to be maximum control values then set scale
 *  as 0.5
 */

void
ControllerSetChannelScale( vexrtController *c, int channel, float scale )
{
    if( (scale < 0.0) || (scale > 1.0))
        return;

    if( (channel < 0) || (channel > kControlChannelCount) )
        return;

    c->ctlscale[ channel ] = scale;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get a control value                                            */
/** @param[in]  c The controller                                               */
/** @param[in]  channel The controller channel                                 */
/** @returns    The control channel value                                      */
/*-----------------------------------------------------------------------------*/

int
ControllerGetControlValue( vexrtController *c, int channel )
{
    int ctl_p = channel;
    int ctl_n = channel+1;
    int ctlvalue = 0;

    // Do we have an enabling button ?
    if( c->ctlebl >= 0 )
        {
        // we use a bitmask to indicate if the enable button should be used for this
        // channel.
        // this channel masked ?
        if( c->ctleblmask & (1<<channel) )
            {
            if( vexRT[ c->ctlebl ] != c->ctleblstate )
                return(0);
            }
        }

    // Valid control ?
    if( c->ctl[ctl_p] == (-1) )
            return(0);

    // Read channel
    if( c->ctl[ctl_n] != (-1) )
        {
        // Buttons
        if( vexRT[ c->ctl[ctl_p] ] == 1 )
            ctlvalue = 127    * c->ctlscale[ctl_p];
        else
        if( vexRT[ c->ctl[ctl_n] ] == 1 )
            ctlvalue = (-127) * c->ctlscale[ctl_n];
        else
            ctlvalue = 0;
        }
    else
        {
        // Joystick
        ctlvalue = vexRT[ c->ctl[ctl_p] ];

        if( abs( ctlvalue ) < c->threshold )
            ctlvalue = 0;
        }

    return(ctlvalue);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get a control button state                                     */
/** @param[in]  c The controller                                               */
/** @param[in]  channel The controller channel                                 */
/** @returns    control button state 0 = none, 1 = up, 2 = down, 3 = both      */
/*-----------------------------------------------------------------------------*/

int
ControllerGetControlState( vexrtController *c, int channel )
{
    int ctl_p = channel;
    int ctl_n = channel+1;
    int ctlstate = 0;

    // Do we have an enabling button ?
    if( c->ctlebl >= 0 )
        {
        // we use a bitmask to indicate if the enable button should be used for this
        // channel.
        // this channel masked ?
        if( c->ctleblmask & (1<<channel) )
            {
            if( vexRT[ c->ctlebl ] != c->ctleblstate )
                return(0);
            }
        }

    // Valid control ?
    if( c->ctl[ctl_p] == (-1) )
            return(0);

    // Read channel
    if( c->ctl[ctl_n] != (-1) )
        {
        // Buttons
        if( vexRT[ c->ctl[ctl_p] ] == 1 )
            ctlstate |= 1;

        if( vexRT[ c->ctl[ctl_n] ] == 1 )
            ctlstate |= 2;
        }
    else
        {
        // Joystick - not relevant
        ctlstate = 0;
        }

    return(ctlstate);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get a control value                                            */
/** @param[in]  c The controller                                               */
/** @param[in]  ctl0 Pointer to control variable                               */
/*-----------------------------------------------------------------------------*/

void
ControllerGetControlValues( vexrtController *c, int *ctl0 )
{
    if(ctl0 != NULL)
        *ctl0 = ControllerGetControlValue( c, CTL00_P );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get a control value                                            */
/** @param[in]  c The controller                                               */
/** @param[in]  ctl0 Pointer to control variable                               */
/** @param[in]  ctl1 Pointer to control variable                               */
/*-----------------------------------------------------------------------------*/

void
ControllerGetControlValues( vexrtController *c, int *ctl0, int *ctl1 )
{
    if(ctl0 != NULL)
        *ctl0 = ControllerGetControlValue( c, CTL00_P );
    if(ctl1 != NULL)
        *ctl1 = ControllerGetControlValue( c, CTL01_P );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get a control value                                            */
/** @param[in]  c The controller                                               */
/** @param[in]  ctl0 Pointer to control variable                               */
/** @param[in]  ctl1 Pointer to control variable                               */
/** @param[in]  ctl2 Pointer to control variable                               */
/*-----------------------------------------------------------------------------*/

void
ControllerGetControlValues( vexrtController *c, int *ctl0, int *ctl1, int *ctl2 )
{
    if(ctl0 != NULL)
        *ctl0 = ControllerGetControlValue( c, CTL00_P );
    if(ctl1 != NULL)
        *ctl1 = ControllerGetControlValue( c, CTL01_P );
    if(ctl2 != NULL)
        *ctl2 = ControllerGetControlValue( c, CTL02_P );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get a control value                                            */
/** @param[in]  c The controller                                               */
/** @param[in]  ctl0 Pointer to control variable                               */
/** @param[in]  ctl1 Pointer to control variable                               */
/** @param[in]  ctl2 Pointer to control variable                               */
/** @param[in]  ctl3 Pointer to control variable                               */
/*-----------------------------------------------------------------------------*/

void
ControllerGetControlValues( vexrtController *c, int *ctl0, int *ctl1, int *ctl2, int *ctl3 )
{
    if(ctl0 != NULL)
        *ctl0 = ControllerGetControlValue( c, CTL00_P );
    if(ctl1 != NULL)
        *ctl1 = ControllerGetControlValue( c, CTL01_P );
    if(ctl2 != NULL)
        *ctl2 = ControllerGetControlValue( c, CTL02_P );
    if(ctl3 != NULL)
        *ctl3 = ControllerGetControlValue( c, CTL03_P );
}

#endif // __CTLLIB__
