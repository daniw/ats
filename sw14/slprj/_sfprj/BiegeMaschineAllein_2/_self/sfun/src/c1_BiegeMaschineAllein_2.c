/* Include files */

#include <stddef.h>
#include "blas.h"
#include "BiegeMaschineAllein_2_sfun.h"
#include "c1_BiegeMaschineAllein_2.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "BiegeMaschineAllein_2_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define c1_event_sequencer             (0)
#define c1_event_StartButtonPressed    (1)
#define c1_event_FootpedalPressed      (2)
#define CALL_EVENT                     (-1)
#define c1_IN_NO_ACTIVE_CHILD          ((uint8_T)0U)
#define c1_IN_Init                     ((uint8_T)1U)
#define c1_IN_PreInit                  ((uint8_T)2U)
#define c1_IN_State2                   ((uint8_T)3U)
#define c1_IN_Stop                     ((uint8_T)4U)
#define c1_const_Kp                    (3.0)
#define c1_const_Y1ref                 (55.0)
#define c1_const_Kp1                   (0.1)
#define c1_const_Td1                   (0.1)
#define c1_const_Ti1                   (3000.0)
#define c1_const_Ts                    (0.01)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_debug_family_names[15] = { "up", "ui", "ud", "nargin",
  "nargout", "r", "y", "Kp", "Ti", "Td", "T", "e_old", "ui_old", "u", "e" };

static const char * c1_b_debug_family_names[9] = { "nargin", "nargout",
  "y_target_p", "y_ist_p", "tf_p", "a3p", "a2p", "a1p", "a0p" };

static const char * c1_c_debug_family_names[8] = { "nargin", "nargout", "a3p",
  "a2p", "a1p", "a0p", "tkp", "ysollp" };

/* Function Declarations */
static void initialize_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static void initialize_params_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static void enable_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static void disable_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static void set_sim_state_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance, const mxArray *c1_st);
static void c1_set_sim_state_side_effects_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static void finalize_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static void sf_c1_BiegeMaschineAllein_2(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance);
static void c1_chartstep_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static void initSimStructsc1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_e, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_info_helper(const mxArray **c1_info);
static const mxArray *c1_emlrt_marshallOut(char * c1_u);
static const mxArray *c1_b_emlrt_marshallOut(uint32_T c1_u);
static void c1_PID(SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance,
                   real_T c1_r, real_T c1_y, real_T c1_b_Kp, real_T c1_Ti,
                   real_T c1_Td, real_T c1_T, real_T c1_e_old, real_T c1_ui_old,
                   real_T *c1_u, real_T *c1_e);
static void c1_TrajectoryPreparation(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, real_T c1_y_target_p, real_T c1_y_ist_p, real_T c1_tf_p,
  real_T *c1_a3p, real_T *c1_a2p, real_T *c1_a1p, real_T *c1_a0p);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int8_T c1_c_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_d_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static uint8_T c1_e_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_b_tp_PreInit, const char_T *c1_identifier);
static uint8_T c1_f_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_g_emlrt_marshallIn
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance, const mxArray
   *c1_b_setSimStateSideEffectsInfo, const char_T *c1_identifier);
static const mxArray *c1_h_emlrt_marshallIn
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance, const mxArray *c1_u,
   const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
  real_T *c1_SN;
  real_T *c1_SP;
  real_T *c1_LS;
  real_T *c1_u1;
  real_T *c1_u2;
  real_T *c1_ysoll;
  c1_ysoll = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c1_u2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c1_u1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_LS = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_SP = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_SN = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_doSetSimStateSideEffects = 0U;
  chartInstance->c1_setSimStateSideEffectsInfo = NULL;
  chartInstance->c1_tp_Init = 0U;
  chartInstance->c1_temporalCounter_i2 = 0U;
  chartInstance->c1_tp_PreInit = 0U;
  chartInstance->c1_tp_State2 = 0U;
  chartInstance->c1_temporalCounter_i1 = 0U;
  chartInstance->c1_tp_Stop = 0U;
  chartInstance->c1_temporalCounter_i1 = 0U;
  chartInstance->c1_is_active_c1_BiegeMaschineAllein_2 = 0U;
  chartInstance->c1_is_c1_BiegeMaschineAllein_2 = c1_IN_NO_ACTIVE_CHILD;
  chartInstance->c1_e1 = 0.0;
  chartInstance->c1_a0 = 0.0;
  chartInstance->c1_a1 = 0.0;
  chartInstance->c1_a2 = 0.0;
  chartInstance->c1_a3 = 0.0;
  chartInstance->c1_tk = 0.0;
  chartInstance->c1_Kp = 3.0;
  chartInstance->c1_Y1ref = 55.0;
  chartInstance->c1_Kp1 = 0.1;
  chartInstance->c1_Td1 = 0.1;
  chartInstance->c1_Ti1 = 3000.0;
  chartInstance->c1_Ts = 0.01;
  if (!(cdrGetOutputPortReusable(chartInstance->S, 1) != 0)) {
    *c1_SN = 0.0;
  }

  if (!(cdrGetOutputPortReusable(chartInstance->S, 2) != 0)) {
    *c1_SP = 0.0;
  }

  if (!(cdrGetOutputPortReusable(chartInstance->S, 3) != 0)) {
    *c1_LS = 0.0;
  }

  if (!(cdrGetOutputPortReusable(chartInstance->S, 4) != 0)) {
    *c1_u1 = 0.0;
  }

  if (!(cdrGetOutputPortReusable(chartInstance->S, 5) != 0)) {
    *c1_u2 = 0.0;
  }

  if (!(cdrGetOutputPortReusable(chartInstance->S, 6) != 0)) {
    *c1_ysoll = 0.0;
  }
}

static void initialize_params_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
}

static void enable_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_update_debugger_state_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
  uint32_T c1_prevAniVal;
  c1_prevAniVal = _SFD_GET_ANIMATION();
  _SFD_SET_ANIMATION(0U);
  _SFD_SET_HONOR_BREAKPOINTS(0U);
  if (chartInstance->c1_is_active_c1_BiegeMaschineAllein_2 == 1U) {
    _SFD_CC_CALL(CHART_ACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_c1_BiegeMaschineAllein_2 == c1_IN_PreInit) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_c1_BiegeMaschineAllein_2 == c1_IN_Init) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_c1_BiegeMaschineAllein_2 == c1_IN_State2) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_c1_BiegeMaschineAllein_2 == c1_IN_Stop) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c1_sfEvent);
  }

  _SFD_SET_ANIMATION(c1_prevAniVal);
  _SFD_SET_HONOR_BREAKPOINTS(1U);
  _SFD_ANIMATE();
}

static const mxArray *get_sim_state_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_hoistedGlobal;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_hoistedGlobal;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_hoistedGlobal;
  real_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_hoistedGlobal;
  real_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  real_T c1_g_hoistedGlobal;
  real_T c1_g_u;
  const mxArray *c1_h_y = NULL;
  real_T c1_h_hoistedGlobal;
  real_T c1_h_u;
  const mxArray *c1_i_y = NULL;
  real_T c1_i_hoistedGlobal;
  real_T c1_i_u;
  const mxArray *c1_j_y = NULL;
  real_T c1_j_hoistedGlobal;
  real_T c1_j_u;
  const mxArray *c1_k_y = NULL;
  real_T c1_k_hoistedGlobal;
  real_T c1_k_u;
  const mxArray *c1_l_y = NULL;
  real_T c1_l_hoistedGlobal;
  real_T c1_l_u;
  const mxArray *c1_m_y = NULL;
  uint8_T c1_m_hoistedGlobal;
  uint8_T c1_m_u;
  const mxArray *c1_n_y = NULL;
  uint8_T c1_n_hoistedGlobal;
  uint8_T c1_n_u;
  const mxArray *c1_o_y = NULL;
  uint8_T c1_o_hoistedGlobal;
  uint8_T c1_o_u;
  const mxArray *c1_p_y = NULL;
  uint8_T c1_p_hoistedGlobal;
  uint8_T c1_p_u;
  const mxArray *c1_q_y = NULL;
  real_T *c1_LS;
  real_T *c1_SN;
  real_T *c1_SP;
  real_T *c1_u1;
  real_T *c1_u2;
  real_T *c1_ysoll;
  c1_ysoll = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c1_u2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c1_u1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_LS = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_SP = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_SN = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(16), FALSE);
  c1_hoistedGlobal = *c1_LS;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *c1_SN;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = *c1_SP;
  c1_c_u = c1_c_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_hoistedGlobal = *c1_u1;
  c1_d_u = c1_d_hoistedGlobal;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_e_hoistedGlobal = *c1_u2;
  c1_e_u = c1_e_hoistedGlobal;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 4, c1_f_y);
  c1_f_hoistedGlobal = *c1_ysoll;
  c1_f_u = c1_f_hoistedGlobal;
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_g_hoistedGlobal = chartInstance->c1_a0;
  c1_g_u = c1_g_hoistedGlobal;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 6, c1_h_y);
  c1_h_hoistedGlobal = chartInstance->c1_a1;
  c1_h_u = c1_h_hoistedGlobal;
  c1_i_y = NULL;
  sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 7, c1_i_y);
  c1_i_hoistedGlobal = chartInstance->c1_a2;
  c1_i_u = c1_i_hoistedGlobal;
  c1_j_y = NULL;
  sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_i_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 8, c1_j_y);
  c1_j_hoistedGlobal = chartInstance->c1_a3;
  c1_j_u = c1_j_hoistedGlobal;
  c1_k_y = NULL;
  sf_mex_assign(&c1_k_y, sf_mex_create("y", &c1_j_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 9, c1_k_y);
  c1_k_hoistedGlobal = chartInstance->c1_e1;
  c1_k_u = c1_k_hoistedGlobal;
  c1_l_y = NULL;
  sf_mex_assign(&c1_l_y, sf_mex_create("y", &c1_k_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 10, c1_l_y);
  c1_l_hoistedGlobal = chartInstance->c1_tk;
  c1_l_u = c1_l_hoistedGlobal;
  c1_m_y = NULL;
  sf_mex_assign(&c1_m_y, sf_mex_create("y", &c1_l_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 11, c1_m_y);
  c1_m_hoistedGlobal = chartInstance->c1_is_active_c1_BiegeMaschineAllein_2;
  c1_m_u = c1_m_hoistedGlobal;
  c1_n_y = NULL;
  sf_mex_assign(&c1_n_y, sf_mex_create("y", &c1_m_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 12, c1_n_y);
  c1_n_hoistedGlobal = chartInstance->c1_is_c1_BiegeMaschineAllein_2;
  c1_n_u = c1_n_hoistedGlobal;
  c1_o_y = NULL;
  sf_mex_assign(&c1_o_y, sf_mex_create("y", &c1_n_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 13, c1_o_y);
  c1_o_hoistedGlobal = chartInstance->c1_temporalCounter_i1;
  c1_o_u = c1_o_hoistedGlobal;
  c1_p_y = NULL;
  sf_mex_assign(&c1_p_y, sf_mex_create("y", &c1_o_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 14, c1_p_y);
  c1_p_hoistedGlobal = chartInstance->c1_temporalCounter_i2;
  c1_p_u = c1_p_hoistedGlobal;
  c1_q_y = NULL;
  sf_mex_assign(&c1_q_y, sf_mex_create("y", &c1_p_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 15, c1_q_y);
  sf_mex_assign(&c1_st, c1_y, FALSE);
  return c1_st;
}

static void set_sim_state_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T *c1_LS;
  real_T *c1_SN;
  real_T *c1_SP;
  real_T *c1_u1;
  real_T *c1_u2;
  real_T *c1_ysoll;
  c1_ysoll = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c1_u2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c1_u1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_LS = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_SP = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_SN = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_u = sf_mex_dup(c1_st);
  *c1_LS = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 0)),
    "LS");
  *c1_SN = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 1)),
    "SN");
  *c1_SP = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 2)),
    "SP");
  *c1_u1 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 3)),
    "u1");
  *c1_u2 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 4)),
    "u2");
  *c1_ysoll = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u,
    5)), "ysoll");
  chartInstance->c1_a0 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 6)), "a0");
  chartInstance->c1_a1 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 7)), "a1");
  chartInstance->c1_a2 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 8)), "a2");
  chartInstance->c1_a3 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 9)), "a3");
  chartInstance->c1_e1 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 10)), "e1");
  chartInstance->c1_tk = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 11)), "tk");
  chartInstance->c1_is_active_c1_BiegeMaschineAllein_2 = c1_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 12)),
     "is_active_c1_BiegeMaschineAllein_2");
  chartInstance->c1_is_c1_BiegeMaschineAllein_2 = c1_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 13)),
     "is_c1_BiegeMaschineAllein_2");
  chartInstance->c1_temporalCounter_i1 = c1_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 14)), "temporalCounter_i1");
  chartInstance->c1_temporalCounter_i2 = c1_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 15)), "temporalCounter_i2");
  sf_mex_assign(&chartInstance->c1_setSimStateSideEffectsInfo,
                c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 16)), "setSimStateSideEffectsInfo"), TRUE);
  sf_mex_destroy(&c1_u);
  chartInstance->c1_doSetSimStateSideEffects = 1U;
  c1_update_debugger_state_c1_BiegeMaschineAllein_2(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void c1_set_sim_state_side_effects_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
  if (chartInstance->c1_doSetSimStateSideEffects != 0) {
    if (chartInstance->c1_is_c1_BiegeMaschineAllein_2 == c1_IN_Init) {
      chartInstance->c1_tp_Init = 1U;
      if (sf_mex_sub(chartInstance->c1_setSimStateSideEffectsInfo,
                     "setSimStateSideEffectsInfo", 1, 2) == 0.0) {
        chartInstance->c1_temporalCounter_i2 = 0U;
      }
    } else {
      chartInstance->c1_tp_Init = 0U;
    }

    if (chartInstance->c1_is_c1_BiegeMaschineAllein_2 == c1_IN_PreInit) {
      chartInstance->c1_tp_PreInit = 1U;
    } else {
      chartInstance->c1_tp_PreInit = 0U;
    }

    if (chartInstance->c1_is_c1_BiegeMaschineAllein_2 == c1_IN_State2) {
      chartInstance->c1_tp_State2 = 1U;
      if (sf_mex_sub(chartInstance->c1_setSimStateSideEffectsInfo,
                     "setSimStateSideEffectsInfo", 1, 4) == 0.0) {
        chartInstance->c1_temporalCounter_i1 = 0U;
      }
    } else {
      chartInstance->c1_tp_State2 = 0U;
    }

    if (chartInstance->c1_is_c1_BiegeMaschineAllein_2 == c1_IN_Stop) {
      chartInstance->c1_tp_Stop = 1U;
      if (sf_mex_sub(chartInstance->c1_setSimStateSideEffectsInfo,
                     "setSimStateSideEffectsInfo", 1, 5) == 0.0) {
        chartInstance->c1_temporalCounter_i1 = 0U;
      }
    } else {
      chartInstance->c1_tp_Stop = 0U;
    }

    chartInstance->c1_doSetSimStateSideEffects = 0U;
  }
}

static void finalize_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
  sf_mex_destroy(&chartInstance->c1_setSimStateSideEffectsInfo);
}

static void sf_c1_BiegeMaschineAllein_2(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance)
{
  int8_T *c1_sequencer;
  int8_T *c1_StartButtonPressed;
  int8_T *c1_FootpedalPressed;
  c1_FootpedalPressed = (int8_T *)*(ssGetInputPortSignalPtrs(chartInstance->S, 2)
    + 2);
  c1_StartButtonPressed = (int8_T *)*(ssGetInputPortSignalPtrs(chartInstance->S,
    2) + 1);
  c1_sequencer = (int8_T *)*(ssGetInputPortSignalPtrs(chartInstance->S, 2) + 0);
  c1_set_sim_state_side_effects_c1_BiegeMaschineAllein_2(chartInstance);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  if (*c1_sequencer == 1) {
    if ((int16_T)chartInstance->c1_temporalCounter_i1 < 1) {
      chartInstance->c1_temporalCounter_i1++;
    }

    if (chartInstance->c1_temporalCounter_i2 < 31U) {
      chartInstance->c1_temporalCounter_i2++;
    }

    chartInstance->c1_sfEvent = c1_event_sequencer;
    _SFD_CE_CALL(EVENT_BEFORE_BROADCAST_TAG, c1_event_sequencer,
                 chartInstance->c1_sfEvent);
    c1_chartstep_c1_BiegeMaschineAllein_2(chartInstance);
    _SFD_CE_CALL(EVENT_AFTER_BROADCAST_TAG, c1_event_sequencer,
                 chartInstance->c1_sfEvent);
    if ((int16_T)chartInstance->c1_temporalCounter_i1 == 1) {
      chartInstance->c1_temporalCounter_i1 = 0U;
    }
  }

  if (*c1_StartButtonPressed == 1) {
    chartInstance->c1_sfEvent = c1_event_StartButtonPressed;
    _SFD_CE_CALL(EVENT_BEFORE_BROADCAST_TAG, c1_event_StartButtonPressed,
                 chartInstance->c1_sfEvent);
    c1_chartstep_c1_BiegeMaschineAllein_2(chartInstance);
    _SFD_CE_CALL(EVENT_AFTER_BROADCAST_TAG, c1_event_StartButtonPressed,
                 chartInstance->c1_sfEvent);
  }

  if (*c1_FootpedalPressed == 1) {
    chartInstance->c1_sfEvent = c1_event_FootpedalPressed;
    _SFD_CE_CALL(EVENT_BEFORE_BROADCAST_TAG, c1_event_FootpedalPressed,
                 chartInstance->c1_sfEvent);
    c1_chartstep_c1_BiegeMaschineAllein_2(chartInstance);
    _SFD_CE_CALL(EVENT_AFTER_BROADCAST_TAG, c1_event_FootpedalPressed,
                 chartInstance->c1_sfEvent);
  }

  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_BiegeMaschineAllein_2MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c1_chartstep_c1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
  boolean_T c1_temp;
  boolean_T c1_out;
  boolean_T c1_b_out;
  real_T c1_a0p;
  real_T c1_a1p;
  real_T c1_a2p;
  real_T c1_a3p;
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  real_T c1_r;
  real_T c1_y;
  real_T c1_e_old;
  real_T c1_ui_old;
  real_T c1_e;
  real_T c1_u;
  real_T *c1_y1;
  real_T *c1_ysoll;
  real_T *c1_u1;
  c1_ysoll = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c1_u1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_y1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  if (chartInstance->c1_is_active_c1_BiegeMaschineAllein_2 == 0U) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
    chartInstance->c1_is_active_c1_BiegeMaschineAllein_2 = 1U;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
    chartInstance->c1_is_c1_BiegeMaschineAllein_2 = c1_IN_PreInit;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_PreInit = 1U;
  } else {
    switch (chartInstance->c1_is_c1_BiegeMaschineAllein_2) {
     case c1_IN_Init:
      CV_CHART_EVAL(0, 0, 1);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 2U,
                   chartInstance->c1_sfEvent);
      c1_temp = (_SFD_CCP_CALL(2U, 0, chartInstance->c1_sfEvent ==
                  c1_event_sequencer != 0U, chartInstance->c1_sfEvent) != 0);
      if (c1_temp) {
        c1_temp = (_SFD_CCP_CALL(2U, 1, (int16_T)
                    chartInstance->c1_temporalCounter_i2 >= 20 != 0U,
                    chartInstance->c1_sfEvent) != 0);
      }

      c1_out = (CV_TRANSITION_EVAL(2U, (int32_T)c1_temp) != 0);
      if (c1_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
        chartInstance->c1_tp_Init = 0U;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
        chartInstance->c1_is_c1_BiegeMaschineAllein_2 = c1_IN_State2;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
        chartInstance->c1_temporalCounter_i1 = 0U;
        chartInstance->c1_tp_State2 = 1U;
      } else {
        _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 0U,
                     chartInstance->c1_sfEvent);
      }

      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
      break;

     case c1_IN_PreInit:
      CV_CHART_EVAL(0, 0, 2);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 1U,
                   chartInstance->c1_sfEvent);
      c1_b_out = (CV_TRANSITION_EVAL(1U, (int32_T)_SFD_CCP_CALL(1U, 0,
        chartInstance->c1_sfEvent == c1_event_FootpedalPressed != 0U,
        chartInstance->c1_sfEvent)) != 0);
      if (c1_b_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 1U, chartInstance->c1_sfEvent);
        chartInstance->c1_tp_PreInit = 0U;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
        chartInstance->c1_is_c1_BiegeMaschineAllein_2 = c1_IN_Init;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
        chartInstance->c1_temporalCounter_i2 = 0U;
        chartInstance->c1_tp_Init = 1U;
        chartInstance->c1_tk = 0.0;
        c1_TrajectoryPreparation(chartInstance, 200.0, *c1_y1, 2.0, &c1_a3p,
          &c1_a2p, &c1_a1p, &c1_a0p);
        chartInstance->c1_a3 = c1_a3p;
        chartInstance->c1_a2 = c1_a2p;
        chartInstance->c1_a1 = c1_a1p;
        chartInstance->c1_a0 = c1_a0p;
      } else {
        _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 2U,
                     chartInstance->c1_sfEvent);
      }

      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c1_sfEvent);
      break;

     case c1_IN_State2:
      CV_CHART_EVAL(0, 0, 3);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_State2 = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
      chartInstance->c1_is_c1_BiegeMaschineAllein_2 = c1_IN_Stop;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c1_sfEvent);
      chartInstance->c1_temporalCounter_i1 = 0U;
      chartInstance->c1_tp_Stop = 1U;
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c1_sfEvent);
      break;

     case c1_IN_Stop:
      CV_CHART_EVAL(0, 0, 4);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 4U,
                   chartInstance->c1_sfEvent);
      if (chartInstance->c1_sfEvent == c1_event_sequencer) {
        if ((int16_T)chartInstance->c1_temporalCounter_i1 == 1) {
          CV_STATE_EVAL(4, 0, 2);
          *c1_ysoll = 200.0;
        } else {
          CV_STATE_EVAL(4, 0, 1);
        }
      } else {
        CV_STATE_EVAL(4, 0, 1);
      }

      if (chartInstance->c1_sfEvent == c1_event_sequencer) {
        if ((int16_T)chartInstance->c1_temporalCounter_i1 == 1) {
          CV_STATE_EVAL(4, 1, 2);
          c1_hoistedGlobal = *c1_ysoll;
          c1_b_hoistedGlobal = *c1_y1;
          c1_c_hoistedGlobal = chartInstance->c1_e1;
          c1_d_hoistedGlobal = *c1_u1;
          c1_r = c1_hoistedGlobal;
          c1_y = c1_b_hoistedGlobal;
          c1_e_old = c1_c_hoistedGlobal;
          c1_ui_old = c1_d_hoistedGlobal;
          c1_PID(chartInstance, c1_r, c1_y, c1_const_Kp1, c1_const_Ti1,
                 c1_const_Td1, c1_const_Ts, c1_e_old, c1_ui_old, &c1_u, &c1_e);
        } else {
          CV_STATE_EVAL(4, 1, 1);
        }
      } else {
        CV_STATE_EVAL(4, 1, 1);
      }

      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c1_sfEvent);
      break;

     default:
      CV_CHART_EVAL(0, 0, 0);
      chartInstance->c1_is_c1_BiegeMaschineAllein_2 = c1_IN_NO_ACTIVE_CHILD;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
      break;
    }
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_BiegeMaschineAllein_2
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
  chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_e, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_e), &c1_thisId);
  sf_mex_destroy(&c1_e);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_e;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
  chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)chartInstanceVoid;
  c1_e = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_e), &c1_thisId);
  sf_mex_destroy(&c1_e);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_BiegeMaschineAllein_2_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_createstruct("structure", 2, 19, 1),
                FALSE);
  c1_info_helper(&c1_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(const mxArray **c1_info)
{
  const mxArray *c1_rhs0 = NULL;
  const mxArray *c1_lhs0 = NULL;
  const mxArray *c1_rhs1 = NULL;
  const mxArray *c1_lhs1 = NULL;
  const mxArray *c1_rhs2 = NULL;
  const mxArray *c1_lhs2 = NULL;
  const mxArray *c1_rhs3 = NULL;
  const mxArray *c1_lhs3 = NULL;
  const mxArray *c1_rhs4 = NULL;
  const mxArray *c1_lhs4 = NULL;
  const mxArray *c1_rhs5 = NULL;
  const mxArray *c1_lhs5 = NULL;
  const mxArray *c1_rhs6 = NULL;
  const mxArray *c1_lhs6 = NULL;
  const mxArray *c1_rhs7 = NULL;
  const mxArray *c1_lhs7 = NULL;
  const mxArray *c1_rhs8 = NULL;
  const mxArray *c1_lhs8 = NULL;
  const mxArray *c1_rhs9 = NULL;
  const mxArray *c1_lhs9 = NULL;
  const mxArray *c1_rhs10 = NULL;
  const mxArray *c1_lhs10 = NULL;
  const mxArray *c1_rhs11 = NULL;
  const mxArray *c1_lhs11 = NULL;
  const mxArray *c1_rhs12 = NULL;
  const mxArray *c1_lhs12 = NULL;
  const mxArray *c1_rhs13 = NULL;
  const mxArray *c1_lhs13 = NULL;
  const mxArray *c1_rhs14 = NULL;
  const mxArray *c1_lhs14 = NULL;
  const mxArray *c1_rhs15 = NULL;
  const mxArray *c1_lhs15 = NULL;
  const mxArray *c1_rhs16 = NULL;
  const mxArray *c1_lhs16 = NULL;
  const mxArray *c1_rhs17 = NULL;
  const mxArray *c1_lhs17 = NULL;
  const mxArray *c1_rhs18 = NULL;
  const mxArray *c1_lhs18 = NULL;
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("mtimes"), "name", "name", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c1_rhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m!common_checks"),
                  "context", "context", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c1_rhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("mrdivide"), "name", "name", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1373306508U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1319729966U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c1_rhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("rdivide"), "name", "name", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c1_rhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c1_rhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c1_rhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_div"), "name", "name", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363713866U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c1_rhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("mpower"), "name", "name", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c1_rhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c1_rhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("ismatrix"), "name", "name", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1331304858U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c1_rhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("power"), "name", "name", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c1_rhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c1_rhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c1_rhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1358186140U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c1_rhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("floor"), "name", "name", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c1_rhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c1_rhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286818726U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c1_rhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c1_rhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("mtimes"), "name", "name", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c1_rhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c1_lhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs18), "lhs", "lhs",
                  18);
  sf_mex_destroy(&c1_rhs0);
  sf_mex_destroy(&c1_lhs0);
  sf_mex_destroy(&c1_rhs1);
  sf_mex_destroy(&c1_lhs1);
  sf_mex_destroy(&c1_rhs2);
  sf_mex_destroy(&c1_lhs2);
  sf_mex_destroy(&c1_rhs3);
  sf_mex_destroy(&c1_lhs3);
  sf_mex_destroy(&c1_rhs4);
  sf_mex_destroy(&c1_lhs4);
  sf_mex_destroy(&c1_rhs5);
  sf_mex_destroy(&c1_lhs5);
  sf_mex_destroy(&c1_rhs6);
  sf_mex_destroy(&c1_lhs6);
  sf_mex_destroy(&c1_rhs7);
  sf_mex_destroy(&c1_lhs7);
  sf_mex_destroy(&c1_rhs8);
  sf_mex_destroy(&c1_lhs8);
  sf_mex_destroy(&c1_rhs9);
  sf_mex_destroy(&c1_lhs9);
  sf_mex_destroy(&c1_rhs10);
  sf_mex_destroy(&c1_lhs10);
  sf_mex_destroy(&c1_rhs11);
  sf_mex_destroy(&c1_lhs11);
  sf_mex_destroy(&c1_rhs12);
  sf_mex_destroy(&c1_lhs12);
  sf_mex_destroy(&c1_rhs13);
  sf_mex_destroy(&c1_lhs13);
  sf_mex_destroy(&c1_rhs14);
  sf_mex_destroy(&c1_lhs14);
  sf_mex_destroy(&c1_rhs15);
  sf_mex_destroy(&c1_lhs15);
  sf_mex_destroy(&c1_rhs16);
  sf_mex_destroy(&c1_lhs16);
  sf_mex_destroy(&c1_rhs17);
  sf_mex_destroy(&c1_lhs17);
  sf_mex_destroy(&c1_rhs18);
  sf_mex_destroy(&c1_lhs18);
}

static const mxArray *c1_emlrt_marshallOut(char * c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c1_u)), FALSE);
  return c1_y;
}

static const mxArray *c1_b_emlrt_marshallOut(uint32_T c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 7, 0U, 0U, 0U, 0), FALSE);
  return c1_y;
}

static void c1_PID(SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance,
                   real_T c1_r, real_T c1_y, real_T c1_b_Kp, real_T c1_Ti,
                   real_T c1_Td, real_T c1_T, real_T c1_e_old, real_T c1_ui_old,
                   real_T *c1_u, real_T *c1_e)
{
  uint32_T c1_debug_family_var_map[15];
  real_T c1_up;
  real_T c1_ui;
  real_T c1_ud;
  real_T c1_nargin = 8.0;
  real_T c1_nargout = 2.0;
  real_T c1_a;
  real_T c1_b;
  real_T c1_A;
  real_T c1_B;
  real_T c1_x;
  real_T c1_b_y;
  real_T c1_b_x;
  real_T c1_c_y;
  real_T c1_d_y;
  real_T c1_b_a;
  real_T c1_b_b;
  real_T c1_e_y;
  real_T c1_b_A;
  real_T c1_c_x;
  real_T c1_d_x;
  real_T c1_f_y;
  real_T c1_c_a;
  real_T c1_c_b;
  real_T c1_g_y;
  real_T c1_c_A;
  real_T c1_b_B;
  real_T c1_e_x;
  real_T c1_h_y;
  real_T c1_f_x;
  real_T c1_i_y;
  _SFD_SET_DATA_VALUE_PTR(29U, c1_e);
  _SFD_SET_DATA_VALUE_PTR(28U, c1_u);
  _SFD_SET_DATA_VALUE_PTR(27U, &c1_ui_old);
  _SFD_SET_DATA_VALUE_PTR(26U, &c1_e_old);
  _SFD_SET_DATA_VALUE_PTR(25U, &c1_T);
  _SFD_SET_DATA_VALUE_PTR(24U, &c1_Td);
  _SFD_SET_DATA_VALUE_PTR(23U, &c1_Ti);
  _SFD_SET_DATA_VALUE_PTR(22U, &c1_b_Kp);
  _SFD_SET_DATA_VALUE_PTR(21U, &c1_y);
  _SFD_SET_DATA_VALUE_PTR(20U, &c1_r);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 15U, 15U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_up, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ui, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ud, 2U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 3U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 4U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_r, 5U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y, 6U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_Kp, 7U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Ti, 8U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Td, 9U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_T, 10U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_e_old, 11U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ui_old, 12U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_u, 13U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_e, 14U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(1, 0);
  _SFD_EML_CALL(1U, chartInstance->c1_sfEvent, 4);
  *c1_e = c1_r - c1_y;
  _SFD_EML_CALL(1U, chartInstance->c1_sfEvent, 7);
  c1_a = c1_b_Kp;
  c1_b = *c1_e;
  c1_up = c1_a * c1_b;
  _SFD_EML_CALL(1U, chartInstance->c1_sfEvent, 10);
  c1_A = c1_T;
  c1_B = c1_Ti;
  c1_x = c1_A;
  c1_b_y = c1_B;
  c1_b_x = c1_x;
  c1_c_y = c1_b_y;
  c1_d_y = c1_b_x / c1_c_y;
  c1_b_a = c1_d_y;
  c1_b_b = *c1_e + c1_e_old;
  c1_e_y = c1_b_a * c1_b_b;
  c1_b_A = c1_e_y;
  c1_c_x = c1_b_A;
  c1_d_x = c1_c_x;
  c1_f_y = c1_d_x / 2.0;
  c1_ui = c1_ui_old + c1_f_y;
  _SFD_EML_CALL(1U, chartInstance->c1_sfEvent, 13);
  c1_c_a = c1_Td;
  c1_c_b = *c1_e - c1_e_old;
  c1_g_y = c1_c_a * c1_c_b;
  c1_c_A = c1_g_y;
  c1_b_B = c1_T;
  c1_e_x = c1_c_A;
  c1_h_y = c1_b_B;
  c1_f_x = c1_e_x;
  c1_i_y = c1_h_y;
  c1_ud = c1_f_x / c1_i_y;
  _SFD_EML_CALL(1U, chartInstance->c1_sfEvent, 15);
  *c1_u = (c1_up + c1_ui) + c1_ud;
  _SFD_EML_CALL(1U, chartInstance->c1_sfEvent, -15);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_UNSET_DATA_VALUE_PTR(29U);
  _SFD_UNSET_DATA_VALUE_PTR(28U);
  _SFD_UNSET_DATA_VALUE_PTR(27U);
  _SFD_UNSET_DATA_VALUE_PTR(26U);
  _SFD_UNSET_DATA_VALUE_PTR(25U);
  _SFD_UNSET_DATA_VALUE_PTR(24U);
  _SFD_UNSET_DATA_VALUE_PTR(23U);
  _SFD_UNSET_DATA_VALUE_PTR(22U);
  _SFD_UNSET_DATA_VALUE_PTR(21U);
  _SFD_UNSET_DATA_VALUE_PTR(20U);
}

static void c1_TrajectoryPreparation(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, real_T c1_y_target_p, real_T c1_y_ist_p, real_T c1_tf_p,
  real_T *c1_a3p, real_T *c1_a2p, real_T *c1_a1p, real_T *c1_a0p)
{
  uint32_T c1_debug_family_var_map[9];
  real_T c1_nargin = 3.0;
  real_T c1_nargout = 4.0;
  real_T c1_b;
  real_T c1_y;
  real_T c1_a;
  real_T c1_b_b;
  real_T c1_b_y;
  real_T c1_A;
  real_T c1_B;
  real_T c1_x;
  real_T c1_c_y;
  real_T c1_b_x;
  real_T c1_d_y;
  real_T c1_c_b;
  real_T c1_e_y;
  real_T c1_b_a;
  real_T c1_d_b;
  real_T c1_f_y;
  real_T c1_c_a;
  real_T c1_e_b;
  real_T c1_g_y;
  real_T c1_b_A;
  real_T c1_b_B;
  real_T c1_c_x;
  real_T c1_h_y;
  real_T c1_d_x;
  real_T c1_i_y;
  _SFD_SET_DATA_VALUE_PTR(36U, c1_a0p);
  _SFD_SET_DATA_VALUE_PTR(35U, c1_a1p);
  _SFD_SET_DATA_VALUE_PTR(34U, c1_a2p);
  _SFD_SET_DATA_VALUE_PTR(32U, c1_a3p);
  _SFD_SET_DATA_VALUE_PTR(33U, &c1_tf_p);
  _SFD_SET_DATA_VALUE_PTR(31U, &c1_y_ist_p);
  _SFD_SET_DATA_VALUE_PTR(30U, &c1_y_target_p);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c1_b_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y_target_p, 2U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y_ist_p, 3U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_tf_p, 4U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_a3p, 5U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_a2p, 6U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_a1p, 7U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_a0p, 8U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(6, 0);
  _SFD_EML_CALL(6U, chartInstance->c1_sfEvent, 3);
  *c1_a0p = c1_y_ist_p;
  _SFD_EML_CALL(6U, chartInstance->c1_sfEvent, 4);
  *c1_a1p = 0.0;
  _SFD_EML_CALL(6U, chartInstance->c1_sfEvent, 5);
  c1_b = c1_y_target_p - c1_y_ist_p;
  c1_y = 3.0 * c1_b;
  c1_a = c1_tf_p;
  c1_b_b = c1_tf_p;
  c1_b_y = c1_a * c1_b_b;
  c1_A = c1_y;
  c1_B = c1_b_y;
  c1_x = c1_A;
  c1_c_y = c1_B;
  c1_b_x = c1_x;
  c1_d_y = c1_c_y;
  *c1_a2p = c1_b_x / c1_d_y;
  _SFD_EML_CALL(6U, chartInstance->c1_sfEvent, 6);
  c1_c_b = c1_y_ist_p - c1_y_target_p;
  c1_e_y = 2.0 * c1_c_b;
  c1_b_a = c1_tf_p;
  c1_d_b = c1_tf_p;
  c1_f_y = c1_b_a * c1_d_b;
  c1_c_a = c1_f_y;
  c1_e_b = c1_tf_p;
  c1_g_y = c1_c_a * c1_e_b;
  c1_b_A = c1_e_y;
  c1_b_B = c1_g_y;
  c1_c_x = c1_b_A;
  c1_h_y = c1_b_B;
  c1_d_x = c1_c_x;
  c1_i_y = c1_h_y;
  *c1_a3p = c1_d_x / c1_i_y;
  _SFD_EML_CALL(6U, chartInstance->c1_sfEvent, -6);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_UNSET_DATA_VALUE_PTR(36U);
  _SFD_UNSET_DATA_VALUE_PTR(35U);
  _SFD_UNSET_DATA_VALUE_PTR(34U);
  _SFD_UNSET_DATA_VALUE_PTR(32U);
  _SFD_UNSET_DATA_VALUE_PTR(33U);
  _SFD_UNSET_DATA_VALUE_PTR(31U);
  _SFD_UNSET_DATA_VALUE_PTR(30U);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int8_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
  chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int8_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 2, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int8_T c1_c_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int8_T c1_y;
  int8_T c1_i0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i0, 1, 2, 0U, 0, 0U, 0);
  c1_y = c1_i0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_sequencer;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int8_T c1_y;
  SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
  chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)chartInstanceVoid;
  c1_sequencer = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_sequencer),
    &c1_thisId);
  sf_mex_destroy(&c1_sequencer);
  *(int8_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
  chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int32_T c1_d_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i1;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i1, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i1;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
  chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  uint8_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
  chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(uint8_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_b_tp_PreInit, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_tp_PreInit),
    &c1_thisId);
  sf_mex_destroy(&c1_b_tp_PreInit);
  return c1_y;
}

static uint8_T c1_f_emlrt_marshallIn(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_tp_PreInit;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  uint8_T c1_y;
  SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
  chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)chartInstanceVoid;
  c1_b_tp_PreInit = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_tp_PreInit),
    &c1_thisId);
  sf_mex_destroy(&c1_b_tp_PreInit);
  *(uint8_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_g_emlrt_marshallIn
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance, const mxArray
   *c1_b_setSimStateSideEffectsInfo, const char_T *c1_identifier)
{
  const mxArray *c1_y = NULL;
  emlrtMsgIdentifier c1_thisId;
  c1_y = NULL;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  sf_mex_assign(&c1_y, c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_setSimStateSideEffectsInfo), &c1_thisId), FALSE);
  sf_mex_destroy(&c1_b_setSimStateSideEffectsInfo);
  return c1_y;
}

static const mxArray *c1_h_emlrt_marshallIn
  (SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance, const mxArray *c1_u,
   const emlrtMsgIdentifier *c1_parentId)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_duplicatearraysafe(&c1_u), FALSE);
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_BiegeMaschineAllein_2InstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_BiegeMaschineAllein_2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(707669869U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(293926459U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3574977460U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1390743781U);
}

mxArray *sf_c1_BiegeMaschineAllein_2_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("oRopURmMEcAFMJjZirXDaB");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_BiegeMaschineAllein_2_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_BiegeMaschineAllein_2_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c1_BiegeMaschineAllein_2(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[7],T\"LS\",},{M[1],M[5],T\"SN\",},{M[1],M[6],T\"SP\",},{M[1],M[8],T\"u1\",},{M[1],M[9],T\"u2\",},{M[1],M[64],T\"ysoll\",},{M[3],M[75],T\"a0\",},{M[3],M[76],T\"a1\",},{M[3],M[77],T\"a2\",},{M[3],M[78],T\"a3\",}}",
    "100 S1x6'type','srcId','name','auxInfo'{{M[3],M[63],T\"e1\",},{M[3],M[79],T\"tk\",},{M[8],M[0],T\"is_active_c1_BiegeMaschineAllein_2\",},{M[9],M[0],T\"is_c1_BiegeMaschineAllein_2\",},{M[11],M[18],T\"temporalCounter_i1\",S'et','os','ct'{{T\"ev\",M1x2[19 69],M[0]}}},{M[11],M[18],T\"temporalCounter_i2\",S'et','os','ct'{{T\"ev\",M[11],M[1]}}}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 16, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_BiegeMaschineAllein_2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
    chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _BiegeMaschineAllein_2MachineNumber_,
           1,
           7,
           4,
           43,
           3,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_BiegeMaschineAllein_2MachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_BiegeMaschineAllein_2MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _BiegeMaschineAllein_2MachineNumber_,
            chartInstance->chartNumber,
            3,
            3,
            3);
          _SFD_SET_DATA_PROPS(0,1,1,0,"y1");
          _SFD_SET_DATA_PROPS(1,1,1,0,"y2");
          _SFD_SET_DATA_PROPS(2,2,0,1,"SN");
          _SFD_SET_DATA_PROPS(3,2,0,1,"SP");
          _SFD_SET_DATA_PROPS(4,2,0,1,"LS");
          _SFD_SET_DATA_PROPS(5,2,0,1,"u1");
          _SFD_SET_DATA_PROPS(6,2,0,1,"u2");
          _SFD_SET_DATA_PROPS(7,7,0,0,"Kp");
          _SFD_SET_DATA_PROPS(8,7,0,0,"Y1ref");
          _SFD_SET_DATA_PROPS(9,0,0,0,"e1");
          _SFD_SET_DATA_PROPS(10,2,0,1,"ysoll");
          _SFD_SET_DATA_PROPS(11,7,0,0,"Kp1");
          _SFD_SET_DATA_PROPS(12,7,0,0,"Td1");
          _SFD_SET_DATA_PROPS(13,7,0,0,"Ti1");
          _SFD_SET_DATA_PROPS(14,7,0,0,"Ts");
          _SFD_SET_DATA_PROPS(15,0,0,0,"a0");
          _SFD_SET_DATA_PROPS(16,0,0,0,"a1");
          _SFD_SET_DATA_PROPS(17,0,0,0,"a2");
          _SFD_SET_DATA_PROPS(18,0,0,0,"a3");
          _SFD_SET_DATA_PROPS(19,0,0,0,"tk");
          _SFD_SET_DATA_PROPS(20,8,0,0,"");
          _SFD_SET_DATA_PROPS(21,8,0,0,"");
          _SFD_SET_DATA_PROPS(22,8,0,0,"");
          _SFD_SET_DATA_PROPS(23,8,0,0,"");
          _SFD_SET_DATA_PROPS(24,8,0,0,"");
          _SFD_SET_DATA_PROPS(25,8,0,0,"");
          _SFD_SET_DATA_PROPS(26,8,0,0,"");
          _SFD_SET_DATA_PROPS(27,8,0,0,"");
          _SFD_SET_DATA_PROPS(28,9,0,0,"");
          _SFD_SET_DATA_PROPS(29,9,0,0,"");
          _SFD_SET_DATA_PROPS(30,8,0,0,"");
          _SFD_SET_DATA_PROPS(31,8,0,0,"");
          _SFD_SET_DATA_PROPS(32,9,0,0,"");
          _SFD_SET_DATA_PROPS(33,8,0,0,"");
          _SFD_SET_DATA_PROPS(34,9,0,0,"");
          _SFD_SET_DATA_PROPS(35,9,0,0,"");
          _SFD_SET_DATA_PROPS(36,9,0,0,"");
          _SFD_SET_DATA_PROPS(37,8,0,0,"");
          _SFD_SET_DATA_PROPS(38,8,0,0,"");
          _SFD_SET_DATA_PROPS(39,8,0,0,"");
          _SFD_SET_DATA_PROPS(40,8,0,0,"");
          _SFD_SET_DATA_PROPS(41,8,0,0,"");
          _SFD_SET_DATA_PROPS(42,9,0,0,"");
          _SFD_EVENT_SCOPE(0,1);
          _SFD_EVENT_SCOPE(1,1);
          _SFD_EVENT_SCOPE(2,1);
          _SFD_STATE_INFO(0,0,0);
          _SFD_STATE_INFO(2,0,0);
          _SFD_STATE_INFO(3,0,0);
          _SFD_STATE_INFO(4,0,0);
          _SFD_STATE_INFO(1,0,2);
          _SFD_STATE_INFO(5,0,2);
          _SFD_STATE_INFO(6,0,2);
          _SFD_CH_SUBSTATE_COUNT(4);
          _SFD_CH_SUBSTATE_DECOMP(0);
          _SFD_CH_SUBSTATE_INDEX(0,0);
          _SFD_CH_SUBSTATE_INDEX(1,2);
          _SFD_CH_SUBSTATE_INDEX(2,3);
          _SFD_CH_SUBSTATE_INDEX(3,4);
          _SFD_ST_SUBSTATE_COUNT(0,0);
          _SFD_ST_SUBSTATE_COUNT(2,0);
          _SFD_ST_SUBSTATE_COUNT(3,0);
          _SFD_ST_SUBSTATE_COUNT(4,0);
        }

        _SFD_CV_INIT_CHART(4,1,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(2,0,0,0,0,0,NULL,NULL);
        }

        {
          static unsigned int sStartDecMap[] = { 18, 88, 164 };

          static unsigned int sEndDecMap[] = { 27, 97, 173 };

          _SFD_CV_INIT_STATE(3,0,0,0,0,3,&(sStartDecMap[0]),&(sEndDecMap[0]));
        }

        {
          static unsigned int sStartDecMap[] = { 16, 54 };

          static unsigned int sEndDecMap[] = { 25, 63 };

          _SFD_CV_INIT_STATE(4,0,0,0,0,2,&(sStartDecMap[0]),&(sEndDecMap[0]));
        }

        {
          _SFD_CV_INIT_STATE(1,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(5,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(6,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        {
          static unsigned int sStartGuardMap[] = { 0 };

          static unsigned int sEndGuardMap[] = { 16 };

          static int sPostFixPredicateTree[] = { 0 };

          _SFD_CV_INIT_TRANS(1,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,
                             &(sPostFixPredicateTree[0]));
        }

        {
          static unsigned int sStartGuardMap[] = { 9, 0 };

          static unsigned int sEndGuardMap[] = { 18, 19 };

          static int sPostFixPredicateTree[] = { 0, 1, -3 };

          _SFD_CV_INIT_TRANS(2,2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),3,
                             &(sPostFixPredicateTree[0]));
        }

        _SFD_CV_INIT_TRANS(3,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(1,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(1,0,"PID",0,-1,233);
        _SFD_CV_INIT_EML(6,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(6,0,"TrajectoryPreparation",0,-1,195);
        _SFD_CV_INIT_EML(5,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(5,0,"TrajectoryComputation",0,-1,104);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(19,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(21,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(22,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(23,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(24,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(25,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(26,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(27,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(28,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(29,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(30,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(31,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(32,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(33,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(34,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(35,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(36,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(37,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(38,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(39,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(40,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(41,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(42,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(20,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(21,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(22,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(23,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(24,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(25,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(26,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(27,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(28,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(29,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(30,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(31,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(32,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(33,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(34,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(35,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(36,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(37,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(38,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(39,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(40,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(41,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(42,(void *)(NULL));

        {
          real_T *c1_y1;
          real_T *c1_y2;
          real_T *c1_SN;
          real_T *c1_SP;
          real_T *c1_LS;
          real_T *c1_u1;
          real_T *c1_u2;
          real_T *c1_ysoll;
          c1_ysoll = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
          c1_u2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
          c1_u1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c1_LS = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c1_SP = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c1_SN = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c1_y2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c1_y1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c1_y1);
          _SFD_SET_DATA_VALUE_PTR(1U, c1_y2);
          _SFD_SET_DATA_VALUE_PTR(2U, c1_SN);
          _SFD_SET_DATA_VALUE_PTR(3U, c1_SP);
          _SFD_SET_DATA_VALUE_PTR(4U, c1_LS);
          _SFD_SET_DATA_VALUE_PTR(5U, c1_u1);
          _SFD_SET_DATA_VALUE_PTR(6U, c1_u2);
          _SFD_SET_DATA_VALUE_PTR(7U, &chartInstance->c1_Kp);
          _SFD_SET_DATA_VALUE_PTR(8U, &chartInstance->c1_Y1ref);
          _SFD_SET_DATA_VALUE_PTR(9U, &chartInstance->c1_e1);
          _SFD_SET_DATA_VALUE_PTR(10U, c1_ysoll);
          _SFD_SET_DATA_VALUE_PTR(11U, &chartInstance->c1_Kp1);
          _SFD_SET_DATA_VALUE_PTR(12U, &chartInstance->c1_Td1);
          _SFD_SET_DATA_VALUE_PTR(13U, &chartInstance->c1_Ti1);
          _SFD_SET_DATA_VALUE_PTR(14U, &chartInstance->c1_Ts);
          _SFD_SET_DATA_VALUE_PTR(15U, &chartInstance->c1_a0);
          _SFD_SET_DATA_VALUE_PTR(16U, &chartInstance->c1_a1);
          _SFD_SET_DATA_VALUE_PTR(17U, &chartInstance->c1_a2);
          _SFD_SET_DATA_VALUE_PTR(18U, &chartInstance->c1_a3);
          _SFD_SET_DATA_VALUE_PTR(19U, &chartInstance->c1_tk);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _BiegeMaschineAllein_2MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "nUVxilHShddPysgpQB5nxD";
}

static void sf_opaque_initialize_c1_BiegeMaschineAllein_2(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_BiegeMaschineAllein_2InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_BiegeMaschineAllein_2
    ((SFc1_BiegeMaschineAllein_2InstanceStruct*) chartInstanceVar);
  initialize_c1_BiegeMaschineAllein_2((SFc1_BiegeMaschineAllein_2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_BiegeMaschineAllein_2(void *chartInstanceVar)
{
  enable_c1_BiegeMaschineAllein_2((SFc1_BiegeMaschineAllein_2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c1_BiegeMaschineAllein_2(void *chartInstanceVar)
{
  disable_c1_BiegeMaschineAllein_2((SFc1_BiegeMaschineAllein_2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c1_BiegeMaschineAllein_2(void *chartInstanceVar)
{
  sf_c1_BiegeMaschineAllein_2((SFc1_BiegeMaschineAllein_2InstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_BiegeMaschineAllein_2
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_BiegeMaschineAllein_2
    ((SFc1_BiegeMaschineAllein_2InstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_BiegeMaschineAllein_2();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c1_BiegeMaschineAllein_2(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_BiegeMaschineAllein_2();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_BiegeMaschineAllein_2
    ((SFc1_BiegeMaschineAllein_2InstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_BiegeMaschineAllein_2(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c1_BiegeMaschineAllein_2(S);
}

static void sf_opaque_set_sim_state_c1_BiegeMaschineAllein_2(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c1_BiegeMaschineAllein_2(S, st);
}

static void sf_opaque_terminate_c1_BiegeMaschineAllein_2(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_BiegeMaschineAllein_2InstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_BiegeMaschineAllein_2_optimization_info();
    }

    finalize_c1_BiegeMaschineAllein_2((SFc1_BiegeMaschineAllein_2InstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_BiegeMaschineAllein_2
    ((SFc1_BiegeMaschineAllein_2InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_BiegeMaschineAllein_2(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_BiegeMaschineAllein_2
      ((SFc1_BiegeMaschineAllein_2InstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_BiegeMaschineAllein_2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_BiegeMaschineAllein_2_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,1,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,6);
    }

    ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=6; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(875247370U));
  ssSetChecksum1(S,(2720155697U));
  ssSetChecksum2(S,(2625579671U));
  ssSetChecksum3(S,(3211580193U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_BiegeMaschineAllein_2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Stateflow");
  }
}

static void mdlStart_c1_BiegeMaschineAllein_2(SimStruct *S)
{
  SFc1_BiegeMaschineAllein_2InstanceStruct *chartInstance;
  chartInstance = (SFc1_BiegeMaschineAllein_2InstanceStruct *)utMalloc(sizeof
    (SFc1_BiegeMaschineAllein_2InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_BiegeMaschineAllein_2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 0;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_BiegeMaschineAllein_2;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c1_BiegeMaschineAllein_2_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_BiegeMaschineAllein_2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_BiegeMaschineAllein_2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_BiegeMaschineAllein_2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_BiegeMaschineAllein_2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
