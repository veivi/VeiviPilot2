#include <ctype.h>
#include <AP_Progmem/AP_Progmem.h>
#include "Command.h"
#include "NVState.h"
#include "Logging.h"
#include "Math.h"
#include "PPM.h"
#include "Objects.h"

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
  { "rmix", c_rmix, e_float, &vpParam.r_Mix },
  { "tmix", c_tmix, e_float, &vpParam.t_Mix, &vpParam.t_Expo },
  { "edefl", c_edefl, e_angle90, &vpParam.elevDefl },
  { "eneutral", c_eneutral, e_angle90, &vpParam.elevNeutral },
  { "takeoff", c_takeoff, e_percent, &vpParam.takeoffTrim },
  { "adefl", c_adefl, e_angle90, &vpParam.aileDefl },
  { "aneutral", c_aneutral, e_angle90, &vpParam.aileNeutral, &vpParam.aile2Neutral },
  { "rdefl", c_rdefl, e_angle90, &vpParam.rudderDefl },
  { "rneutral", c_rneutral, e_angle90, &vpParam.rudderNeutral },
  { "sdefl", c_sdefl, e_angle90, &vpParam.steerDefl },
  { "sneutral", c_sneutral, e_angle90, &vpParam.steerNeutral },
  { "fdefl", c_fdefl, e_angle90, &vpParam.flapDefl },
  { "fneutral", c_fneutral,
    e_angle90, &vpParam.flapNeutral, &vpParam.flap2Neutral },
  { "bdefl", c_bdefl, e_angle90, &vpParam.brakeDefl },
  { "bneutral", c_bneutral, e_angle90, &vpParam.brakeNeutral },
  { "cdefl", c_cdefl, e_angle90, &vpParam.canardDefl },
  { "cneutral", c_cneutral, e_angle90, &vpParam.canardNeutral },
  { "vdefl", c_vdefl, e_angle90, &vpParam.vertDefl },
  { "vneutral", c_vneutral, e_angle90, &vpParam.vertNeutral },
  { "hdefl", c_hdefl, e_angle90, &vpParam.horizDefl },
  { "hneutral", c_hneutral, e_angle90, &vpParam.horizNeutral },
  { "roll_k", c_roll_k, e_float, &vpParam.roll_C, &vpParam.expo },
  { "servorate", c_servorate, e_float, &vpParam.servoRate },
  { "col_ab", c_col_ab, e_float, &vpParam.cL_A, &vpParam.cL_B, &vpParam.cL_C, &vpParam.cL_D, &vpParam.cL_E },
  { "col_max", c_col_max, e_float, &vpParam.cL_apex, &vpParam.alphaMax },
  { "climb", c_climb, e_angle, &vpParam.maxPitch },
  { "weight", c_weight, e_float, &vpParam.weightDry },
  { "fuel", c_fuel, e_float, &vpParam.fuel },
  { "thrust", c_thrust, e_float, &vpParam.thrust },
  { "virtual", c_virtual, e_bool, &vpParam.virtualOnly },
  { "flaperon", c_elevon, e_bool, &vpParam.flaperon },
  { "margin", c_margin, e_percent, &vpParam.thresholdMargin },
  { "smargin", c_smargin, e_percent, &vpParam.stallMargin },
  { "slope", c_slope, e_angle, &vpParam.glideSlope },
  { "offset", c_offset, e_angle, &vpParam.offset },
  { "wow", c_wow, e_bool, &vpParam.wowCalibrated },
  { "wheels", c_wheels, e_bool, &vpParam.haveGear },
  { "floor", c_floor, e_int16, &vpParam.floor },
  { "map", c_map, e_map, &vpParam.functionMap },
  { "stall", c_stall },
  { "peak", c_peak },
  { "max", c_max },
  { "zl", c_zl },
  { "scale", c_scale },
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
  { "gauge", c_gauge },
  { "stamp", c_stamp },
  { "arm", c_arm },
  { "disarm", c_disarm },
  { "test", c_test },
  { "talk", c_talk },
  { "defaults", c_defaults },
  { "ailetrim", c_atrim },
  { "elevtrim", c_etrim },
  { "ruddertrim", c_rtrim },
  { "rollrate", c_rollrate },
  { "calibrate", c_calibrate },  
  { "curve", c_curve },
  { "gear", c_gear },
  { "fault", c_fault },
  { "function", c_function },
  { "", c_invalid }
};

//
// Command interpreter
//

char *parse(char *ptr)
{
  while(*ptr && !isblank(*ptr))
    ptr++;

  if(*ptr) {
    *ptr++ = '\0';
    while(*ptr && isblank(*ptr))
      ptr++;
  }

  return ptr;
}

void printCoeffElement(float y0, float y1, float x, float v)
{
    consoleNote("");
    consolePrint(x);
    consoleTab(10);

    const int col1 = 78, col0 = 10+(col1-10)*-y0/(y1-y0);
    int y = col0 + (col1-col0) * v / y1;

    if(y < col0) {
      consoleTab(y);
      consolePrint("*");
      consoleTab(col0);
      consolePrint("|");
      consoleTab(col1);
      consolePrintLn("|");
    } else if(y > col0) {
      consoleTab(col0);
      consolePrint("|");
      consoleTab(y);
      if(y < col1) {
	consolePrint("*");
	consoleTab(col1);
	consolePrintLn("|");
      } else
	consolePrintLn("*");
    } else {
      consoleTab(col0);
      consolePrint("*");
      consoleTab(col1);
      consolePrintLn("|");
    }
}

void executeCommand(char *buf)
{
  while(*buf && isblank(*buf))
    buf++;
  
  consolePrint_P(PSTR("// % "));
  consolePrintLn(buf);

  if(!*buf || buf[0] == '/') {
    gaugeCount = 0;
    calibStop(nvState.rxMin, nvState.rxCenter, nvState.rxMax);
    return;
  } else if(atoi(buf) > 0) {
    gaugeCount = 1;
    gaugeVariable[0] = atoi(buf);
    return;
  }
  
  int numParams = 0;
  float param[maxParams];
  const char *paramText[maxParams];

  for(int i = 0; i < maxParams; i++)
    param[i] = 0.0;

  char *parsePtr = buf;
  
  while(*(parsePtr = parse(parsePtr))) {
    if(numParams < maxParams) {
      paramText[numParams] = parsePtr;
      param[numParams] = atof(paramText[numParams]);
      numParams++;
    }
  }
  
  int matches = 0, j = 0;
  struct Command command;
  
  while(1) {
    struct Command cache;
  
    memcpy_P(&cache, &commands[j++], sizeof(cache));

    if(cache.token == c_invalid)
      break;
    
    if(!strncmp(buf, cache.name, strlen(buf))) {
      command = cache;
      matches++;
    }
  }
  
  if(matches < 1) {
    consolePrint_P(PSTR("Command not recognized: \""));
    consolePrint(buf);
    consolePrintLn("\"");
    
  } else if(matches > 1) {
    consolePrint_P(PSTR("Ambiguos command: \""));
    consolePrint(buf);
    consolePrintLn("\"");
    
  } else if(command.var[0]) {
    //
    // Simple variable
    //
    
    for(int i = 0; i < numParams && command.var[i]; i++) {
      switch(command.varType) {
      case e_string:
	strncpy((char*) command.var[i], paramText[i], NAME_LEN-1);
	break;
      
      case e_uint16:
	*((uint16_t*) command.var[i]) = param[i];
	break;
      
      case e_int16:
	*((int16_t*) command.var[i]) = param[i];
	break;
      
      case e_int8:
	*((int8_t*) command.var[i]) = param[i];
	break;
      
      case e_float:
	*((float*) command.var[i]) = param[i];
	break;

      case e_percent:
	*((float*) command.var[i]) = param[i]/100;
	break;

      case e_angle:
	*((float*) command.var[i]) = param[i]/RADIAN;
	break;

      case e_angle90:
	*((float*) command.var[i]) = param[i]/90;
	break;
	
      case e_bool:
	*((bool*) command.var[i]) = param[i];
	break;

      case e_map:
	for(int k = 0; k < MAX_SERVO; k++)
	  ((uint8_t*) command.var[i])[k] = param[i+k];
	break;
      }
    }
  } else {
    //
    // Complex
    //

    float offset = 0.0;
    
    switch(command.token) {
    case c_atrim:
      vpParam.aileNeutral += vpParam.aileDefl*param[0];
      break;
      
    case c_etrim:
      vpParam.elevNeutral += vpParam.elevDefl*param[0];
      break;
      
    case c_rtrim:
      vpParam.rudderNeutral += vpParam.rudderDefl*param[0];
      break;
      
    case c_arm:
      vpStatus.armed = true;
      break;
    
    case c_disarm:
      vpStatus.armed = false;
      consoleNoteLn_P(PSTR("We're DISARMED"));
      break;
    
    case c_talk:
      vpStatus.silent = false;
      consoleNoteLn_P(PSTR("Hello world"));
      break;
    
    case c_test:
      if(numParams > 0)
	logTestSet(param[0]);

      consoleNote_P(PSTR("Current test channel = "));
      consolePrintLn(nvState.testNum);
      break;

    case c_gear:
      if(numParams > 0)
	gearSel = param[0];
      break;
      
    case c_calibrate:
      consoleNoteLn_P(PSTR("Receiver calibration STARTED"));
      calibStart();
      break;

    case c_rollrate:
      if(numParams > 0) {
	vpParam.roll_C
	  = param[0]/RADIAN/powf(vpDerived.minimumIAS, stabilityAileExp2_c);
	consoleNote_P(PSTR("Roll rate K = "));
	consolePrintLn(vpParam.roll_C);
	storeNVState();
      }
      break;
          
    case c_alpha:
      if(numParams > 0)
	offset = param[0];
      
      vpParam.alphaRef +=
	(int16_t) ((1L<<16) * (vpFlight.alpha - offset / RADIAN) / CIRCLE);
      consoleNoteLn_P(PSTR("Alpha calibrated"));
      break;

    case c_gauge:
      if(numParams < 1) {
	gaugeCount = 1;
	gaugeVariable[0] = 1;
      } else {
	gaugeCount = numParams;
	
	for(int i = 0; i < numParams; i++)
	  gaugeVariable[i] = param[i];
      }
      break;
	
    case c_store:
      consoleNoteLn_P(PSTR("Params & NV state stored"));
      storeNVState();
      storeParams();
      backupParams();
      break;

    case c_defaults:
      consoleNoteLn_P(PSTR("Default params restored"));
      defaultParams();
      break;

    case c_dump:
      logDumpBinary();
      break;

    case c_fault:
      if(numParams > 0)
	vpStatus.fault = param[0];
      else
	vpStatus.fault = 0;
      break;

    case c_scale:
      if(numParams > 0) {
	vpParam.i_Ku_C *= param[0];
	vpParam.i_Tu *= param[0];
	vpParam.s_Ku_C *= param[0];
	vpParam.s_Tu *= param[0];
	vpParam.cL_A *= param[0];
	vpParam.cL_B *= param[0];
	vpParam.cL_C *= param[0];
	vpParam.cL_D *= param[0];
	vpParam.cL_E *= param[0];
      }
      break;
    
    case c_stamp:
      if(numParams > 0) {
	nvState.logStamp = param[0];
	storeNVState();
      }
      consoleNote_P(PSTR("Current log stamp is "));
      consolePrintLn(nvState.logStamp);  
      break;

    case c_model:
      if(numParams > 0) {
	if(param[0] > maxModels()-1)
	  param[0] = maxModels()-1;
	setModel(param[0], true);
	storeNVState();
      } else { 
	consoleNote_P(PSTR("Current model is "));
	consolePrintLn(nvState.model); 
      }
      break;
    
    case c_trim:
      if(numParams > 0)
	vpControl.elevTrim = param[0]/100;
      consoleNote_P(PSTR("Current elev trim(%) = "));
      consolePrintLn(vpControl.elevTrim*100); 
      break;
      
    case c_params:
     consoleNote_P(PSTR("SETTINGS (MODEL "));
      consolePrint(nvState.model);
      consolePrintLn(")");
      printParams();
      break;

    case c_delete:
      if(numParams > 0) {
	if(param[0] > maxModels()-1)
	  param[0] = maxModels()-1;
	deleteModel(param[0]);
      }
      break;

    case c_curve:
      consoleNoteLn_P(PSTR("Feed-forward curve"));
  
      for(float aR = -1; aR <= 1; aR += 0.07)
	printCoeffElement(-1, 1, vpParam.alphaMax*aR*RADIAN, alphaPredictInverse(vpParam.alphaMax*aR));

      consoleNoteLn_P(PSTR("Inverse feed-forward curve"));
  
      for(float e = 1; e >= -1; e -= 0.07)
	printCoeffElement(-vpParam.alphaMax/2, vpParam.alphaMax, e, alphaPredict(e));

      consoleNoteLn_P(PSTR("Coeff of lift"));
  
      for(float aR = -0.3; aR <= 1.02; aR += 0.05)
	printCoeffElement(-0.2, 1, vpParam.alphaMax*aR*RADIAN,
			  coeffOfLift(vpParam.alphaMax*aR)/vpDerived.maxCoeffOfLift);
      break;
      
    case c_clear:
      logClear();
      break;

    case c_init:
      // logInit();
      break;

    case c_stop:
      logDisable();
      break;

    case c_start:
      logEnable();
      break;

    case c_log:
      vpMode.loggingSuppressed = false;
      break;

    case c_zl:
      if(numParams > 0) {
	vpParam.cL_B = vpDerived.maxCoeffOfLift/(vpParam.alphaMax - param[0]/RADIAN);
	vpParam.cL_A = -vpParam.cL_B*param[0]/RADIAN;
      }
      break;
      
    case c_peak:
      if(numParams > 0)
	vpParam.cL_B =
	  (1+param[0])*(vpDerived.maxCoeffOfLift - vpParam.cL_A)/vpParam.alphaMax;
      break;
      
    case c_stall:
      if(numParams > 0) {
	vpParam.cL_apex = G * vpDerived.totalMass / dynamicPressure(param[0]);
	if(numParams > 1)
	  vpParam.alphaMax = param[1]/RADIAN;
      }
      break;
      
    case c_max:
      if(numParams > 0)
	vpParam.alphaMax = param[0]/RADIAN;
      break;
      
    case c_report:
      consoleNote_P(PSTR("Idle avg = "));
      consolePrintLn(idleAvg*100,1);
      consoleNote_P(PSTR("PPM frequency = "));
      consolePrint(ppmFreq);
      consolePrint_P(PSTR(" channels = "));
      consolePrintLn(ppmNumChannels);
      consoleNote_P(PSTR("Sim link frequency = "));
      consolePrintLn(simInputFreq);
      consoleNote_P(PSTR("Alpha = "));
      consolePrint(vpFlight.alpha*RADIAN);
      consolePrint_P(PSTR(" (field = "));
      consolePrint(fieldStrength*100);
      consolePrint_P(PSTR("%)"));
      if(vpStatus.alphaFailed)
	consolePrintLn_P(PSTR(" FAIL"));
      else
	consolePrintLn_P(PSTR(" OK"));

      consoleNoteLn_P(PSTR("Sensor entropy"));
      consoleNote_P(PSTR("  Alpha = "));
      consolePrint(alphaEntropyAcc.output());
      consolePrint_P(PSTR("  IAS = "));
      consolePrintLn(iasEntropyAcc.output());

      consoleNote_P(PSTR("Warning flags :"));
      if(pciWarn)
	consolePrint_P(PSTR(" SPURIOUS_PCINT"));
      if(ppmWarnShort)
	consolePrint_P(PSTR(" PPM_SHORT"));
      if(ppmWarnSlow)
	consolePrint_P(PSTR(" PPM_SLOW"));
      if(eepromDevice.warning())
	consolePrint_P(PSTR(" EEPROM_FAILED"));
      if(vpStatus.pitotFailed)
	consolePrint_P(PSTR(" IAS_FAILED"));
      
      consolePrintLn("");

      consoleNote_P(PSTR("Log write bandwidth = "));
      consolePrint(logBandWidth);
      consolePrintLn_P(PSTR(" bytes/sec"));
      
      break;
      
    case c_reset:
      pciWarn = ppmWarnShort = ppmWarnSlow = false;
      consoleNoteLn_P(PSTR("Warning flags reset"));
      break;

    case c_function:
      if(numParams > 1 && param[0] >= 0 && param[0] < MAX_SERVO) {
	function_t fn = fn_invalid;
	
	switch(paramText[1][0]) {
	case 'L':
	  fn = fn_leftaileron;
	  break;
	case 'R':
	  fn = fn_rightaileron;
	  break;
	case 'c':
	  fn = fn_leftcanard;
	  break;
	case 'C':
	  fn = fn_rightcanard;
	  break;
	case 'v':
	  fn = fn_leftelevon;
	  break;
	case 'V':
	  fn = fn_rightelevon;
	  break;
	case 'f':
	  fn = fn_leftflap;
	  break;
	case 'F':
	  fn = fn_rightflap;
	  break;
	case 't':
	  fn = fn_lefttail;
	  break;
	case 'T':
	  fn = fn_righttail;
	  break;
	case 'y':
	  fn = fn_leftthrustvert;
	  break;
	case 'Y':
	  fn = fn_rightthrustvert;
	  break;
	case 'x':
	  fn = fn_thrusthoriz;
	  break;
	case 'a':
	  fn = fn_aileron;
	  break;
	case 'e':
	  fn = fn_elevator;
	  break;
	case 'r':
	  fn = fn_rudder;
	  break;
	case 'g':
	  fn = fn_gear;
	  break;
	case 'b':
	  fn = fn_brake;
	  break;
	case 's':
	  fn = fn_steering;
	  break;
	case 'p':
	  fn = fn_throttle;
	  break;
	case '-':
	  fn = fn_null;
	  break;
	}

	if(fn != fn_invalid)
	  vpParam.functionMap[(uint8_t) param[0]] = fn;
	else
	  consoleNoteLn_P(PSTR("Invalid function"));
      } else {
	consoleNoteLn_P(PSTR("SERVO  FUNCTION"));
	consoleNoteLn_P(PSTR("---------------------"));

	for(int i = 0; i < MAX_SERVO; i++) {
	  consoleNote_P(PSTR("  "));
	  consolePrint(i);
	  consoleTab(10);

	  switch(vpParam.functionMap[i]) {
	  case fn_leftaileron:
	    consolePrintLn_P(PSTR("aileron (left)"));
	    break;
	  case fn_rightaileron:
	    consolePrintLn_P(PSTR("aileron (right)"));
	    break;
	  case fn_leftflap:
	    consolePrintLn_P(PSTR("flap (left)"));
	    break;
	  case fn_rightflap:
	    consolePrintLn_P(PSTR("flap (right)"));
	    break;
	  case fn_leftcanard:
	    consolePrintLn_P(PSTR("canard (left)"));
	    break;
	  case fn_rightcanard:
	    consolePrintLn_P(PSTR("canard (right)"));
	    break;
	  case fn_lefttail:
	    consolePrintLn_P(PSTR("tail (left)"));
	    break;
	  case fn_righttail:
	    consolePrintLn_P(PSTR("tail (right)"));
	    break;
	  case fn_leftthrustvert:
	    consolePrintLn_P(PSTR("vertical thrust (left)"));
	    break;
	  case fn_rightthrustvert:
	    consolePrintLn_P(PSTR("vertical thrust (right)"));
	    break;
	  case fn_leftelevon:
	    consolePrintLn_P(PSTR("elevon (left)"));
	    break;
	  case fn_rightelevon:
	    consolePrintLn_P(PSTR("elevon (right)"));
	    break;
	  case fn_aileron:
	    consolePrintLn_P(PSTR("aileron"));
	    break;
	  case fn_elevator:
	    consolePrintLn_P(PSTR("elevator"));
	    break;
	  case fn_rudder:
	    consolePrintLn_P(PSTR("rudder"));
	    break;
	  case fn_throttle:
	    consolePrintLn_P(PSTR("throttle"));
	    break;
	  case fn_gear:
	    consolePrintLn_P(PSTR("landing gear"));
	    break;
	  case fn_steering:
	    consolePrintLn_P(PSTR("nose wheel"));
	    break;
	  case fn_brake:
	    consolePrintLn_P(PSTR("brake"));
	    break;
	  case fn_thrusthoriz:
	    consolePrintLn_P(PSTR("horizontal thrust"));
	    break;
	  case fn_null:
	    consolePrintLn_P(PSTR("---"));
	    break;
	  default:
	    consolePrintLn_P(PSTR("<invalid>"));
	    break;
	  }
	}
      }
      break;

    default:
      consolePrint_P(PSTR("Sorry, command not implemented: \""));
      consolePrint(buf);
      consolePrintLn("\"");
      break;
    }
  }
}
