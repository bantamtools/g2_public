/*
 * settings_watercolorbot_v2.h - settings for the WaterColorBot v2 (http://watercolorbot.com/)
 * This file is part of the g2core project
 *
 * Copyright (c) 2016 Alden S. Hart Jr.
 * Copyright (c) 2016 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/***********************************************************************/
/**** AxiDraw v3 Configuration *****************************************/
/***********************************************************************/

#include "hardware.h"

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to AxiDraw v3 settings"

#define JUNCTION_INTEGRATION_TIME   2.5    // cornering - usually between 0.5 and 2.0 (higher is faster)
#define CHORDAL_TOLERANCE           0.01    // chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE           0                       // 0=off, 1=on
#define HARD_LIMIT_ENABLE           1                       // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1                       // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1                       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0                       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true
#define SPINDLE_SPINUP_DELAY        1.0

#define COOLANT_MIST_POLARITY       1                       // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1                       // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       false

// Communications and reporting settings

#define COMM_MODE                   JSON_MODE               // one of: TEXT_MODE, JSON_MODE
#define XIO_ENABLE_FLOW_CONTROL FLOW_CONTROL_RTS            // FLOW_CONTROL_OFF, FLOW_CONTROL_RTS

#define TEXT_VERBOSITY              TV_VERBOSE              // one of: TV_SILENT, TV_VERBOSE
#define JSON_VERBOSITY              JV_MESSAGES             // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define QUEUE_REPORT_VERBOSITY      QR_OFF                  // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY     SR_FILTERED             // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS        100                     // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS   250                     // milliseconds - set $SV=0 to disable

//#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","bcr","feed","vel","unit","coor","dist","admo","frmo","momo","stat"
#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","feed","vel","momo","stat"

// Alternate SRs
//#define STATUS_REPORT_DEFAULTS "line","vel","mpox","mpoy","mpoz","mpoa","coor","ofsa","ofsx","ofsy","ofsz","dist","unit","stat","homz","homy","homx","momo"
//#define STATUS_REPORT_DEFAULTS "_ts1","_cs1","_es1","_xs1","_fe1","line","posx","posy","posz","vel","stat"
//#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","vel","_cs1","_es1","_xs1","_fe1","_cs2","_es2","_xs2","_fe2"
//#define STATUS_REPORT_DEFAULTS "mpox","mpoy","mpoz","mpoa","ofsx","ofsy","ofsz","ofsa","unit","stat","coor","momo","dist","home","hold","macs","cycs","mots","plan","feed"
//#define STATUS_REPORT_DEFAULTS "line","mpox","mpoy","mpoz","mpoa","ofsx","ofsy","ofsz","ofsa","stat","_cs1","_es1","_fe0","_fe1","_fe2","_fe3"
//#define STATUS_REPORT_DEFAULTS "line","mpox","mpoy","mpoz","stat","_ts2","_ps2","_cs2","_es2","_fe2"
//#define STATUS_REPORT_DEFAULTS "line","mpox","mpoy","mpoz","_cs3","_es3","_fe3","_xs3","_cs2","_es2","_fe2","_xs2","stat"
//#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","frmo","momo","stat","_cs1","_es1","_xs1","_fe1"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE

// *** motor settings ************************************************************************************

#define MOTOR_POWER_MODE            MOTOR_POWERED_IN_CYCLE  // default motor power mode (see cmMotorPowerMode in stepper.h)
#define MOTOR_POWER_TIMEOUT         2.00                    // motor power timeout in seconds

#define KINEMATICS KINE_CORE_XY                             // X and Y MUST use the same settings!

#define A_B_POWER_LEVEL 0.7
#define A_B_MICROSTEPS 64
#define JERK_MAX    3000

#define M1_MOTOR_MAP                AXIS_COREXY_A           // 1ma
#define M1_STEP_ANGLE               1.8                     // 1sa
#define M1_TRAVEL_PER_REV           40                      // 1tr
#define M1_MICROSTEPS               A_B_MICROSTEPS          // 1mi        1,2,4,8
#define M1_POLARITY                 0                       // 1po        0=normal, 1=reversed
#define M1_POWER_MODE               MOTOR_POWER_MODE        // 1pm        standard
#define M1_POWER_LEVEL              A_B_POWER_LEVEL         // 1pl

#define M2_MOTOR_MAP                AXIS_COREXY_B
#define M2_STEP_ANGLE               1.8
#define M2_TRAVEL_PER_REV           40
#define M2_MICROSTEPS               A_B_MICROSTEPS
#define M2_POLARITY                 0
#define M2_POWER_MODE               MOTOR_POWER_MODE
#define M2_POWER_LEVEL              A_B_POWER_LEVEL

#if MOTORS == 3
// gQuadratic
// #warning Autodetected gQuadratic settings
#define M3_MOTOR_MAP                AXIS_Z
                                                            // This "stepper" is a hobby servo. Note that all hobby
                                                            //   servo settings are per full servo range, instead of
                                                            //   per revolution.
#define M3_STEP_ANGLE               1.8                     // hobby servos are simulated with 200 "full steps"
#define M3_TRAVEL_PER_REV           26                      // this is actually the full travel of the servo, not
                                                            //   necessarily covering a revolution
#define M3_MICROSTEPS               32                      // the max step resolution for a hobby servo is 1/32
#define M3_POLARITY                 1
#define M3_POWER_MODE               MOTOR_ALWAYS_POWERED
#define M3_POWER_LEVEL              0.50                    // this is ignored
#endif

#if MOTORS == 6
// gQuintic
#warning Autodetected gQuintic settings
#define M6_MOTOR_MAP                AXIS_Z
// This "stepper" is a hobby servo. Note that all hobby
//   servo settings are per full servo range, instead of
//   per revolution.
#define M6_STEP_ANGLE               1.8                     // hobby servos are simulated with 200 "full steps"
#define M6_TRAVEL_PER_REV           26                      // this is actually the full travel of the servo, not
//   necessarily covering a revolution
#define M6_MICROSTEPS               32                      // the max step resolution for a hobby servo is 1/32
#define M6_POLARITY                 1
#define M6_POWER_MODE               MOTOR_ALWAYS_POWERED
#define M6_POWER_LEVEL              0.50                    // this is ignored
#endif

// *** axis settings **********************************************************************************


#define X_AXIS_MODE                 AXIS_STANDARD           // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX              15000                   // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX              15000          // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN                0                       // xtn  minimum travel - used by soft limits and homing
#define X_TRAVEL_MAX                400                     // xtm  maximum travel - used by soft limits and homing
#define X_JERK_MAX                  JERK_MAX                // xjm
#define X_JERK_HIGH_SPEED           X_JERK_MAX              // xjh
#define X_HOMING_INPUT              1                       // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION          0                       // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY           1000                    // xsv  move in negative direction
#define X_LATCH_VELOCITY            100                     // xlv  mm/min
#define X_LATCH_BACKOFF             10                      // xlb  mm
#define X_ZERO_BACKOFF              2                       // xzb  mm

#define Y_AXIS_MODE                 AXIS_STANDARD
#define Y_VELOCITY_MAX              15000
#define Y_FEEDRATE_MAX              15000
#define Y_TRAVEL_MIN                0
#define Y_TRAVEL_MAX                175
#define Y_JERK_MAX                  JERK_MAX
#define Y_JERK_HIGH_SPEED           Y_JERK_MAX
#define Y_HOMING_INPUT              3
#define Y_HOMING_DIRECTION          0
#define Y_SEARCH_VELOCITY           1000
#define Y_LATCH_VELOCITY            100
#define Y_LATCH_BACKOFF             10
#define Y_ZERO_BACKOFF              2

#define Z_AXIS_MODE                 AXIS_STANDARD
#define Z_VELOCITY_MAX              10000
#define Z_FEEDRATE_MAX              Z_VELOCITY_MAX
#define Z_TRAVEL_MIN                0
#define Z_TRAVEL_MAX                75
#define Z_JERK_MAX                  1000
#define Z_JERK_HIGH_SPEED           Z_JERK_MAX
#define Z_HOMING_INPUT              6
#define Z_HOMING_DIRECTION          1
#define Z_SEARCH_VELOCITY           600
#define Z_LATCH_VELOCITY            100
#define Z_LATCH_BACKOFF             10
#define Z_ZERO_BACKOFF              2

//*** Input / output settings ***
#define DI1_POLARITY                IO_ACTIVE_HIGH
#define DI1_ACTION                  INPUT_ACTION_STOP

#define DI2_POLARITY                IO_ACTIVE_HIGH
#define DI2_ACTION                  INPUT_ACTION_NONE

#define DI3_POLARITY                IO_ACTIVE_HIGH
#define DI3_ACTION                  INPUT_ACTION_NONE

#define DI4_MODE                    IO_ACTIVE_HIGH
#define DI4_ACTION                  INPUT_ACTION_NONE

#define DO1_MODE                    IO_ACTIVE_HIGH
#define DO2_MODE                    IO_ACTIVE_HIGH
#define DO3_MODE                    IO_ACTIVE_HIGH
#define DO4_MODE                    IO_ACTIVE_HIGH
