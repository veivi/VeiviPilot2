#include <AP_Progmem/AP_Progmem.h>
#include "Command.h"
#include "NVState.h"

const struct Command commands[] PROGMEM = {
  { "name", c_name, e_string, &vpParam.name },
  { "as5048b_ref", c_5048b_ref, e_uint16, &vpParam.alphaRef },
  { "at_zn", c_at_zn, e_float, &vpParam.at_Ku, &vpParam.at_Tu },
  { "cc_zn", c_cc_zn, e_float, &vpParam.cc_Ku, &vpParam.cc_Tu },
  { "inner_pid_zn", c_inner_pid_zn,
    e_float, &vpParam.i_Ku_C, &vpParam.i_Tu },
  { "outer_p", c_outer_p, e_float, &vpParam.o_P },
  { "ff", c_ff, e_float, &vpParam.ff_A, &vpParam.ff_B, &vpParam.ff_C },
  { "stabilizer_pid_zn", c_stabilizer_pid_zn,
    e_float, &vpParam.s_Ku_C, &vpParam.s_Tu },
  { "pusher_pid_zn", c_pusher_pid_zn,
    e_float, &vpParam.p_Ku_C, &vpParam.p_Tu },
  { "rmix", c_rmix, e_float, &vpParam.r_Mix },
  { "tmix", c_tmix, e_float, &vpParam.t_Mix, &vpParam.t_Expo },
  { "edefl", c_edefl, e_angle90, &vpParam.elevDefl },
  { "eneutral", c_eneutral, e_angle90, &vpParam.elevNeutral },
  { "takeoff", c_takeoff, e_percent, &vpParam.takeoffTrim },
  { "eservo", c_eservo, e_int8, &vpParam.servoElev },
  { "adefl", c_adefl, e_angle90, &vpParam.aileDefl },
  { "aneutral", c_aneutral, e_angle90, &vpParam.aileNeutral },
  { "aservo", c_aservo, e_int8, &vpParam.servoAile },
  { "rdefl", c_rdefl, e_angle90, &vpParam.rudderDefl },
  { "rneutral", c_rneutral, e_angle90, &vpParam.rudderNeutral },
  { "rservo", c_rservo, e_int8, &vpParam.servoRudder },
  { "sdefl", c_sdefl, e_angle90, &vpParam.steerDefl },
  { "sneutral", c_sneutral, e_angle90, &vpParam.steerNeutral },
  { "sservo", c_sservo, e_int8, &vpParam.servoSteer },
  { "fstep", c_flapstep, e_angle90, &vpParam.flapStep },
  { "fneutral", c_flapneutral,
    e_angle90, &vpParam.flapNeutral, &vpParam.flap2Neutral },
  { "fservo", c_fservo,
    e_int8, &vpParam.servoFlap, &vpParam.servoFlap2 },
  { "bdefl", c_bdefl, e_angle90, &vpParam.brakeDefl },
  { "bneutral", c_bneutral, e_angle90, &vpParam.brakeNeutral },
  { "bservo", c_bservo, e_int8, &vpParam.servoBrake },
  { "gservo", c_gservo, e_int8, &vpParam.servoGear },  
  { "roll_k", c_roll_k, e_float, &vpParam.roll_C },
  { "servorate", c_servorate, e_float, &vpParam.servoRate },
  { "col_ab", c_col_ab, e_float, &vpParam.cL_A, &vpParam.cL_B },
  { "col_max", c_col_max, e_float, &vpParam.cL_max, &vpParam.alphaMax },
  { "climb", c_climb, e_angle, &vpParam.maxPitch },
  { "weight", c_weight, e_float, &vpParam.weightDry },
  { "expo", c_expo, e_float, &vpParam.expo },
  { "fuel", c_fuel, e_float, &vpParam.fuel },
  { "thrust", c_thrust, e_float, &vpParam.thrust },
  { "virtual", c_virtual, e_bool, &vpParam.virtualOnly },
  { "elevon", c_elevon, e_bool, &vpParam.elevon },
  { "vtail", c_vtail, e_bool, &vpParam.veeTail },
  { "tservo", c_tservo, e_int8, &vpParam.servoThrottle },
  { "margin", c_margin, e_percent, &vpParam.thresholdMargin },
  { "smargin", c_smargin, e_percent, &vpParam.stallMargin },
  { "slope", c_slope, e_angle, &vpParam.glideSlope },
  { "offset", c_offset, e_angle, &vpParam.offset },
  { "wow", c_wow, e_bool, &vpParam.wowCalibrated },
  { "wheels", c_wheels, e_bool, &vpParam.haveWheels },
  { "stall", c_stall },
  { "peak", c_peak },
  { "max", c_max },
  { "zl", c_zl },
  { "update", c_update },
  { "beep", c_beep },
  { "trim", c_trim },
  { "ping", c_ping },
  { "model", c_model },
  { "alpha", c_alpha },
  { "dumpz", c_dump },
  { "clear", c_clear },
  { "init", c_init },
  { "store", c_store },
  { "delete", c_delete },
  { "report", c_report },
  { "stop", c_stop },
  { "log", c_log },
  { "start", c_start },
  { "params", c_params },
  { "reset", c_reset },
  { "loop", c_gauge },
  { "gauge", c_gauge },
  { "stamp", c_stamp },
  { "arm", c_arm },
  { "disarm", c_disarm },
  { "test", c_test },
  { "talk", c_talk },
  { "rattle", c_rattle },
  { "defaults", c_defaults },
  { "ailetrim", c_atrim },
  { "elevtrim", c_etrim },
  { "ruddertrim", c_rtrim },
  { "rollrate", c_rollrate },
  { "calibrate", c_calibrate },  
  { "tab_col", c_tab_col },  
  { "tab_elev", c_tab_elev },  
  { "curve", c_curve },
  { "gear", c_gear },
  { "fault", c_fault },
  { "bias", c_bias },
  { "", c_invalid }
};

