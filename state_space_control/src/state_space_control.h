/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: state_space_control.h
 *
 * Code generated for Simulink model 'state_space_control'.
 *
 * Model version                  : 1.18
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Sun Mar 22 09:03:43 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef state_space_control_h_
#define state_space_control_h_
#ifndef state_space_control_COMMON_INCLUDES_
#define state_space_control_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                /* state_space_control_COMMON_INCLUDES_ */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay1_DSTATE[2];         /* '<S1>/Unit Delay1' */
  real_T UnitDelay_DSTATE;             /* '<S1>/Unit Delay' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T r;                            /* '<Root>/r' */
  real_T y;                            /* '<Root>/y' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T uc;                           /* '<Root>/uc' */
} ExtY;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void state_space_control_initialize(void);
extern void state_space_control_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('SpeedControl/state_space_control')    - opens subsystem SpeedControl/state_space_control
 * hilite_system('SpeedControl/state_space_control/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'SpeedControl'
 * '<S1>'   : 'SpeedControl/state_space_control'
 */
#endif                                 /* state_space_control_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
