#ifndef __c1_BiegeMaschineAllein_2_h__
#define __c1_BiegeMaschineAllein_2_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_BiegeMaschineAllein_2InstanceStruct
#define typedef_SFc1_BiegeMaschineAllein_2InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  uint8_T c1_tp_PreInit;
  uint8_T c1_tp_Init;
  uint8_T c1_tp_State2;
  uint8_T c1_tp_Stop;
  boolean_T c1_isStable;
  uint8_T c1_is_active_c1_BiegeMaschineAllein_2;
  uint8_T c1_is_c1_BiegeMaschineAllein_2;
  real_T c1_Kp;
  real_T c1_Y1ref;
  real_T c1_e1;
  real_T c1_Kp1;
  real_T c1_Td1;
  real_T c1_Ti1;
  real_T c1_Ts;
  real_T c1_a0;
  real_T c1_a1;
  real_T c1_a2;
  real_T c1_a3;
  real_T c1_tk;
  uint8_T c1_temporalCounter_i1;
  uint8_T c1_temporalCounter_i2;
  uint8_T c1_doSetSimStateSideEffects;
  const mxArray *c1_setSimStateSideEffectsInfo;
} SFc1_BiegeMaschineAllein_2InstanceStruct;

#endif                                 /*typedef_SFc1_BiegeMaschineAllein_2InstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_BiegeMaschineAllein_2_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_BiegeMaschineAllein_2_get_check_sum(mxArray *plhs[]);
extern void c1_BiegeMaschineAllein_2_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
