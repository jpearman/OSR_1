/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2012                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*        Module:     PstLib.c                                                 */
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
/** @file    PstLib.c
  * @brief   Library to handle control presets
  * @details
  * Standardized presets
  * Stores an array of of presets, a preset position is a signed integer
  * in the range -32768 to 32767 (using long had issues in ROBOTC V3.51)
*//*---------------------------------------------------------------------------*/

// Stop recursive includes
#ifndef __PSTLIB__
#define __PSTLIB__

// 10 presets should be enough
#define kMaxPresets 10

/** @brief Structure to hold preset data
 */
typedef struct {
    short           preset_num;             ///< total presets
    short           preset_ptr;             ///< index of current preset
    int             presets[kMaxPresets];   ///< array of preset data
    short           tolerance;              ///< tolerance when selecting next preset
    } vexPreset;

/*-----------------------------------------------------------------------------*/
/** @brief      Initialize a preset structure                                  */
/** @param[in]  v Pointer to the preset data structure                         */
/*-----------------------------------------------------------------------------*/

void
PresetInit( vexPreset *v )
{
    int     i;

    // no presets in list
    v->preset_num  = 0;
    v->preset_ptr  = 0;

    // default tolerance 25, good for pid controlled arms etc.
    v->tolerance   = 25;

    // clear presets
    for(i=0;i<kMaxPresets;i++)
        v->presets[i] = 0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Add another preset to the preset data structure                */
/** @param[in]  v Pointer to the preset data structure                         */
/** @param[in]  position preset data to add                                    *
/*-----------------------------------------------------------------------------*/
/** Note
 *  Presets should be added in incrementing order in this version
 *  may add sort later.
 */

void
PresetAdd( vexPreset *v, int position )
{
    if( v->preset_num > kMaxPresets )
        return;

    v->presets[ v->preset_num++ ] = position;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Set the preset tolerance                                       */
/** @param[in]  v Pointer to the preset data structure                         */
/** @param[in]  tolerance The tolerance to use                                 */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Set the tolerance used when determining if the current position is already
 *  at a preset position. Set to 1 to force an exact match.
 */

void
PresetSetTolerance( vexPreset *v, short tolerance )
{
    if( tolerance >= 1)
        v->tolerance = tolerance;
    else
        v->tolerance = 1;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get next preset                                                */
/** @param[in]  v Pointer to the preset data structure                         */
/** @param[in]  position Current control position                              */
/** @param[in]  dir next or previous preset                                    */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Based on a given positon determine the next preset position
 *  returns the preset index (not the actual position) if a preset is found
 *  or (-1) on failure
 */

short
PresetGetNextPosition( vexPreset *v, int position, short dir )
{
    int     i;
    int     current_position = position;

    // No action
    v->preset_ptr = (-1);

    // Do we have any ?
    if( v->preset_num == 0 )
        return( v->preset_ptr );

    // see if we are exactly (within tolerance) on a preset
    for(i=0;i<v->preset_num;i++)
        {
        if( abs(v->presets[i] - current_position) < v->tolerance )
            {
            // found, pick next or previous
            if( dir == 1 )
                v->preset_ptr = ( i < (v->preset_num-1) ) ? i+1 : i;
            else
                v->preset_ptr = ( i >= 1 ) ? i-1 : 0;
            return( v->preset_ptr );
            }
        }

    // Ok, not near a curent preset so look for limit
    for(i=0;i<v->preset_num;i++)
        {
        if( v->presets[i] > current_position )
            {
            // found, pick next or previous
            if( dir == 1 )
                v->preset_ptr = i;
            else
                v->preset_ptr = ( i >= 1 ) ? i-1 : (-1);
            return( v->preset_ptr );
            }
        }

    // Beyond the last preset
    if( dir == 1 )
        v->preset_ptr = (-1);
    else
        v->preset_ptr = v->preset_num - 1;

    return( v->preset_ptr );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Select a preset                                                */
/** @param[in]  v Pointer to the preset data structure                         */
/** @param[in]  p preset index                                                 */
/*-----------------------------------------------------------------------------*/
void
PresetSet( vexPreset *v, short p )
{
    v->preset_ptr = p;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get number of presets in the list                              */
/** @param[in]  v Pointer to the preset data structure                         */
/** @returns    number of presets                                              */
/*-----------------------------------------------------------------------------*/
short
PresetGetNum( vexPreset *v )
{
    return( v->preset_num );
}

/*-----------------------------------------------------------------------------*/
/** @brief      Get current preset position based on the current preset pointer*/
/** @param[in]  v Pointer to the preset data structure                         */
/** @returns    The current preset value                                       */
/*-----------------------------------------------------------------------------*/
long
PresetCurrentPosition( vexPreset *v )
{
    if( v->preset_ptr != (-1) )
        return( v->presets[ v->preset_ptr ] );
    else
        return(0);
}

/*-----------------------------------------------------------------------------*/
/** @brief      check that the preset pointer is valid                         */
/** @param[in]  v Pointer to the preset data structure                         */
/** @returns    valid f;ag                                                     */
/*-----------------------------------------------------------------------------*/
int
PresetIsValid( vexPreset *v )
{
    return( v->preset_ptr );
}


#endif  //__PSTLIB__
