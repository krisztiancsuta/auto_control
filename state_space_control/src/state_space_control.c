
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: state_space_control.c
 *
 * Code generated for Simulink model 'state_space_control'.
 *
 * Model version                  : 1.20
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Wed May  6 11:47:08 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "state_space_control.h"
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void state_space_control_step(void)
{
  real_T rtb_Gain;
  real_T rtb_Sum2_idx_0;
  real_T rtb_Sum2_idx_1;

  /* UnitDelay: '<S1>/Unit Delay' */
  rtb_Gain = rtDW.UnitDelay_DSTATE;

  /* Sum: '<S1>/Sum2' incorporates:
   *  Gain: '<S1>/F_e'
   *  Gain: '<S1>/G_e'
   *  Gain: '<S1>/H_e'
   *  Inport: '<Root>/y'
   *  UnitDelay: '<S1>/Unit Delay1'
   */
  rtb_Sum2_idx_0 = ((0.83527021141127189 * rtDW.UnitDelay1_DSTATE[0] +
                     0.0046481326473157532 * rtDW.UnitDelay1_DSTATE[1]) +
                    0.16194090900033858 * rtU.y) +
                   0.0046481326473157532 *
                       rtb_Gain;
  rtb_Sum2_idx_1 = ((-1.3147667432727737 * rtDW.UnitDelay1_DSTATE[0] +
                     0.99268355301132349 * rtDW.UnitDelay1_DSTATE[1]) +
                    1.3191566114659796 * rtU.y) +
                   -0.007316446988676506 *
                       rtb_Gain;

  /* Sum: '<S1>/Sum1' incorporates:
   *  Gain: '<S1>/Gain'
   *  Gain: '<S1>/Nu'
   *  Inport: '<Root>/r'
   *  Sum: '<S1>/Sum'
   */
  rtb_Gain = ((rtU.r - rtb_Sum2_idx_0) * 2.970182503012524 + 0.60000000000000853 * rtU.r) - rtb_Sum2_idx_1;

  /* Saturate: '<S1>/Saturation' */
  if (rtb_Gain > 20.0)
  {
    rtb_Gain = 20.0;
  }
  else if (rtb_Gain < -20.0)
  {
    rtb_Gain = -20.0;
  }

  /* End of Saturate: '<S1>/Saturation' */

  /* Outport: '<Root>/uc' */
  rtY.uc = rtb_Gain;

  /* Update for UnitDelay: '<S1>/Unit Delay1' */
  rtDW.UnitDelay1_DSTATE[0] = rtb_Sum2_idx_0;
  rtDW.UnitDelay1_DSTATE[1] = rtb_Sum2_idx_1;

  /* Update for UnitDelay: '<S1>/Unit Delay' */
  rtDW.UnitDelay_DSTATE = rtb_Gain;
}

/* Model initialize function */
void state_space_control_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */