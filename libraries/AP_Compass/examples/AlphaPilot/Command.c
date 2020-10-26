#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "Command.h"
#include "StaP.h"
#include "Objects.h"
#include "Logging.h"
#include "Datagram.h"
#include "Console.h"
#include "Function.h"
#include "RxInput.h"
#include "Math.h"
#include "NVState.h"
#include "MS4525.h"
#include "AS5048B.h"
#include "MainLoop.h"

const struct Command commands[] CS_QUALIFIER = {
  { "name" , c_name, e_string, &vpParam.name },
  { "as5048b_ref", c_5048b_ref, e_uint16, &vpParam.alphaRef },
  { "offset", c_offset, e_angle, &vpParam.alphaOffset },
  { "inner_pid_zn", c_inner_pid_zn,
    e_float, &vpParam.i_Ku_C, &vpParam.i_Tu },
  { "outer_p", c_outer_p, e_float, &vpParam.o_P },
  { "ff", c_ff, e_ff_curve, &vpParam.coeff_FF[0] },
  { "alt_ff", c_ff, e_ff_curve, &vpParam.coeff_FF[1] },
  { "stabilizer_pid_zn", c_stabilizer_pid_zn,
    e_float, &vpParam.s_Ku_C, &vpParam.s_Tu },
  { "rudder_pid_zn", c_rudder_pid_zn,
    e_float, &vpParam.r_Ku_C, &vpParam.r_Tu },
  { "rmix", c_rmix, e_float, &vpParam.r_Mix },
  { "tmix", c_tmix, e_float, &vpParam.t_Mix, &vpParam.t_Expo },
  { "idle", c_idle, e_float, &vpParam.idle },
  { "lag", c_lag, e_float, &vpParam.lag },
  { "dimension", c_dimension, e_float, &vpParam.dimension },
  { "edefl", c_edefl, e_angle90, &vpParam.elevDefl },
  { "takeoff", c_takeoff, e_percent, &vpParam.takeoffTrim },
  { "adefl", c_adefl, e_angle90, &vpParam.aileDefl },
  { "rdefl", c_rdefl, e_angle90, &vpParam.rudderDefl },
  { "sdefl", c_sdefl, e_angle90, &vpParam.steerDefl },
  { "strim", c_strim, e_angle90, &vpParam.steerTrim },
  { "spark", c_park, e_angle90, &vpParam.steerPark },
  { "fdefl", c_fdefl, e_angle90, &vpParam.flapDefl },
  { "bdefl", c_bdefl, e_angle90, &vpParam.brakeDefl },
  { "cdefl", c_cdefl, e_angle90, &vpParam.canardDefl },
  { "cgain", c_cgain, e_float, &vpParam.canardGain, &vpParam.canardGainD },
  { "vdefl", c_vdefl, e_angle90, &vpParam.vertDefl },
  { "hdefl", c_hdefl, e_angle90, &vpParam.horizDefl },
  { "roll_k", c_roll_k, e_float, &vpParam.roll_C, &vpParam.roll_Expo },
  { "servorate", c_servorate, e_float, &vpParam.servoRate },
  { "col_ab", c_col, e_col_curve, &vpParam.coeff_CoL[0] },
  { "alt_col_ab", c_col, e_col_curve, &vpParam.coeff_CoL[1] },
  { "max", c_max, e_angle, &vpParam.alphaMax[0], &vpParam.alphaMax[1] },
  { "climb", c_climb, e_angle, &vpParam.maxPitch },
  { "weight", c_weight, e_float, &vpParam.weightDry },
  { "fuel", c_fuel, e_float, &vpParam.fuel },
  { "density", c_density, e_float, &vpParam.fuelDensity },
  { "battery", c_battery, e_float, &vpParam.battery },
  { "thrust", c_thrust, e_float, &vpParam.thrust },
  { "virtual", c_virtual, e_bool, &vpParam.virtualOnly },
  { "sensor", c_sensor, e_bool, &vpParam.sensorOrient },
  { "margin", c_margin, e_percent, &vpParam.thresholdMargin },
  { "smargin", c_smargin, e_percent, &vpParam.shakerMargin },
  { "pmargin", c_pmargin, e_angle, &vpParam.pushMargin },
  { "yawdamper", c_yawdamper, e_float, &vpParam.yd_C },
  { "wow", c_wow, e_bool, &vpParam.wowCalibrated },
  { "wheels", c_wheels, e_bool, &vpParam.haveGear },
  { "map", c_map, e_map, &vpParam.functionMap },
  { "nmap", c_nmap, e_nmap, &vpParam.neutral },
  { "flare", c_flare, e_float, &vpParam.flare },
  { "flow", c_flow, e_fuel_curve, &vpParam.coeff_Flow },
  { "doordelay", c_doordelay, e_uint16, &vpParam.doorDelay },
  { "geardelay", c_geardelay, e_uint16, &vpParam.gearDelay },
  { "gearspeed", c_gearspeed, e_uint16, &vpParam.gearSpeed },
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
  { "gauge", c_gauge },
  { "stamp", c_stamp },
  { "arm", c_arm },
  { "disarm", c_disarm },
  { "test", c_test },
  { "talk", c_talk },
  { "defaults", c_defaults },
  { "rollrate", c_rollrate },
  { "calibrate", c_calibrate },  
  { "curve", c_curve },
  { "gear", c_gear },
  { "fault", c_fault },
  { "function", c_function },
  { "neutral", c_neutral },
  { "read", c_read },
  { "reset", c_reset },
  { "boot", c_boot },
  { "memtest", c_memtest },
  { "scale", c_scale },
  { "launch", c_launch },
  { "ready", c_ready },
  { "elevator", c_elevator },
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
    consolePrintF(x);
    consoleTab(10);

    const int col1 = 78, col0 = 10+(col1-10)*-y0/(y1-y0);
    int y = col0 + (col1-col0) * v / y1;

    if(y < col0) {
      if(y >= 10) {
	consoleTab(y);
	consolePrint("*");
      }
      consoleTab(col0);
      consolePrint("|");
      consoleTab(col1);
      consolePrintLn("|");
    } else if(y > col0) {
      consoleTab(col0);
      consolePrint("|");
      consoleTab(MIN(col1,y));
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
  
  consolePrint_P(CS_STRING("// % "));
  consolePrintLn(buf);

  if(!*buf || buf[0] == '/') {
    gaugeCount = 0;
    inputCalibStop();
    return;
  } else if(atoi(buf) > 0) {
    gaugeCount = 1;
    gaugeVariable[0] = atoi(buf);
    return;
  }
  
  int numParams = 0;
  float param[MAX_PARAMS];
  const char *paramText[MAX_PARAMS];
  int i = 0;

  for(i = 0; i < MAX_PARAMS; i++)
    param[i] = 0.0;

  char *parsePtr = buf;
  
  while(*(parsePtr = parse(parsePtr))) {
    if(numParams < MAX_PARAMS) {
      paramText[numParams] = parsePtr;
      param[numParams] = atof(paramText[numParams]);
      numParams++;
    }
  }
  
  int matches = 0, j = 0;
  struct Command command;
  
  while(1) {
    struct Command cache;
  
    CS_MEMCPY(&cache, &commands[j++], sizeof(cache));

    if(cache.token == c_invalid)
      break;
    
    if(!strncmp(buf, cache.name, strlen(buf))) {
      command = cache;
      matches++;
    }
  }
  
  if(matches < 1) {
    consolePrint_P(CS_STRING("Command not recognized: \""));
    consolePrint(buf);
    consolePrintLn("\"");
    
  } else if(matches > 1) {
    consolePrint_P(CS_STRING("Ambiguos command: \""));
    consolePrint(buf);
    consolePrintLn("\"");
    
  } else if(command.var[0]) {
    //
    // A simple variable
    //

    int k = 0;
    
    for(i = 0; i < numParams && command.var[i]; i++) {
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
	for(k = 0; k < MAX_SERVO; k++)
	  ((uint8_t*) command.var[i])[k] = param[i+k];
	break;
	
      case e_nmap:
	for(k = 0; k < MAX_SERVO; k++)
	  ((float*) command.var[i])[k] = param[i+k]/90;
	break;
	
      case e_col_curve:
	for(k = 0; k < CoL_degree+1; k++)
	  ((float*) command.var[i])[k] = param[i+k];
	break;

      case e_ff_curve:
	for(k = 0; k < FF_degree+1; k++)
	  ((float*) command.var[i])[k] = param[i+k];
	break;

      case e_fuel_curve:
	for(k = 0; k < FuelFlow_degree+1; k++)
	  ((float*) command.var[i])[k] = param[i+k];
	break;
      }
    }

    derivedInvalidate();
  } else {
    //
    // A complex command
    //

    float offset = 0.0;
    
    switch(command.token) {
    case c_reset:
      consoleNote("Reboot...");
      consoleFlush();
      stap_reboot(false);
      break;

    case c_boot:
      consoleNote("Reboot into bootloader...");
      consoleFlush();
      vpDelayMillis(500);
      datagramTxStart(DG_DISCONNECT);
      datagramTxEnd();
      vpDelayMillis(500);
      stap_reboot(true);
      break;
      
    case c_memtest:
      consoleNoteLn_P(CS_STRING("Log memory test "));
      if(logTest())
	consoleNoteLn_P(CS_STRING("PASSED"));
      else
	consoleNoteLn_P(CS_STRING("FAILED"));
      break;
      
    case c_read:
      if(numParams > 1) {
	uint8_t target = param[0], addr = param[1], data = 0,
	  status = stap_I2cRead(target, &addr, 1, &data, 1);
	consolePrintUI(data);
	consolePrint(", ");
	consolePrintLnUI(status);
      }
      break;
      
    case c_arm:
      vpMode.armed = true;
      break;
    
    case c_disarm:
      vpMode.armed = false;
      consoleNoteLn_P(CS_STRING("We're DISARMED"));
      break;
    
    case c_talk:
      if(numParams > 0)
	vpMode.silent = true;
      else {
	vpMode.silent = false;
	consoleNoteLn_P(CS_STRING("Hello world"));
      }
      break;
    
    case c_test:
      if(numParams > 0) {
	for(i = 0; i < MAX_TESTS; i++) {
	  if(i < numParams)
	    nvState.testNum[i] = param[i];
	  else
	    nvState.testNum[i] = -1;
	}
	  
	storeNVState();
      }

      consoleNote_P(CS_STRING("Current test program = [ "));
      for(i = 0; i < MAX_TESTS; i++) {
	if(nvState.testNum[i] >= 0) {
	  if(i > 0)
	    consolePrint(", ");
	  consolePrintI(nvState.testNum[i]);
	}
      }
      consolePrintLn(" ]");
      break;

    case c_gear:
      if(numParams > 0)
	vpControl.gearSel = param[0];
      break;
      
    case c_calibrate:
      consoleNoteLn_P(CS_STRING("Receiver calibration STARTED"));
      inputCalibStart();
      break;

    case c_rollrate:
      if(numParams > 0) {
	vpParam.roll_C = param[0]/RADIAN/powf(vpDerived.minimumIAS, 1);
	consoleNote_P(CS_STRING("Roll rate K = "));
	consolePrintLnF(vpParam.roll_C);
	vpDerived.valid = false;
      }
      break;
          
    case c_alpha:
      if(numParams > 0)
	offset = param[0];
      
      vpParam.alphaRef +=
	(int16_t) ((1L<<16) * (vpFlight.alpha - offset / RADIAN) / CIRCLE);
      consoleNoteLn_P(CS_STRING("Alpha calibrated"));
      break;

    case c_gauge:
      if(numParams < 1) {
	gaugeCount = 1;
	gaugeVariable[0] = 1;
      } else {
	gaugeCount = numParams;
	
	for(i = 0; i < numParams; i++)
	  gaugeVariable[i] = param[i];
      }
      break;
	
    case c_store:
      consoleNoteLn_P(CS_STRING("Params & NV state stored"));
      storeNVState();
      storeParams();
      backupParams();
      break;

    case c_defaults:
      consoleNoteLn_P(CS_STRING("Default params restored"));
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
    
    case c_stamp:
      if(numParams > 0) {
	nvState.logStamp = param[0];
	storeNVState();
      }
      consoleNote_P(CS_STRING("Current log stamp is "));
      consolePrintLnUI(nvState.logStamp);  
      break;

    case c_model:
      if(numParams > 0) {
	if(param[0] > maxModels()-1)
	  param[0] = maxModels()-1;
	setModel(param[0], true);
	storeNVState();
      } else { 
	consoleNote_P(CS_STRING("Current model is "));
	consolePrintLnUI(nvState.model); 
      }
      break;
    
    case c_trim:
      if(numParams > 0)
	vpControl.elevTrim = param[0]/100;
      consoleNote_P(CS_STRING("Current elev trim(%) = "));
      consolePrintLnF(vpControl.elevTrim*100); 
      break;
      
    case c_params:
     consoleNote_P(CS_STRING("SETTINGS (MODEL "));
      consolePrintUI(nvState.model);
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
      consoleNoteLn_P(CS_STRING("Feed-forward curve"));
  
      float aR = 0;

      for(aR = -1; aR < 1.1; aR += 0.1)
	printCoeffElement(-1, 1, vpDerived.maxAlpha*aR*RADIAN, alphaPredictInverse(vpDerived.maxAlpha*aR));

      consoleNoteLn_P(CS_STRING("Coeff of lift"));
  
      for(aR = -0.5; aR < 1.1; aR += 0.1)
	printCoeffElement(-0.3, 1, vpDerived.maxAlpha*aR*RADIAN,
			  coeffOfLift(vpDerived.maxAlpha*aR)/vpDerived.maxCoeffOfLiftLand);
      break;
      
    case c_clear:
      logClear();
      break;

    case c_stop:
      logDisable();
      break;

    case c_start:
      logEnable();
      break;

    case c_log:
      vpMode.dontLog = false;
      break;

    case c_report:
      consoleNote_P(CS_STRING("Free mem "));
      consolePrintUL(stap_memoryFree());
      consolePrintLn_P(CS_STRING(" bytes"));
      
      consoleNote_P(CS_STRING("Load = "));
      consolePrintLnFP(vpStatus.load*100,1);
      consoleNote_P(CS_STRING("PPM frequency = "));
      consolePrintLnF(ppmFreq);
      consoleNote_P(CS_STRING("Sim link frequency = "));
      consolePrintLnF(simInputFreq);
      consoleNote_P(CS_STRING("Alpha = "));
      consolePrintF(vpFlight.alpha*RADIAN);
      consolePrint_P(CS_STRING(" (field = "));
      consolePrintF(fieldStrength*100);
      consolePrint_P(CS_STRING("%)"));
      if(vpStatus.alphaFailed)
	consolePrintLn_P(CS_STRING(" FAIL"));
      else
	consolePrintLn_P(CS_STRING(" OK"));

      consoleNoteLn_P(CS_STRING("Sensor entropy"));
      consoleNote_P(CS_STRING("  Alpha = "));
      consolePrintF(AS5048B_entropy());
      consolePrint_P(CS_STRING("  IAS = "));
      consolePrintLnF(MS4525DO_entropy());

      consoleNote_P(CS_STRING("Warning flags :"));
      if(!inputSourceGood())
	consolePrint_P(CS_STRING(" PPM"));
      if(vpStatus.pitotFailed)
	consolePrint_P(CS_STRING(" IAS_FAILED"));
      
      consolePrintLn("");

      consoleNote_P(CS_STRING("Log write bandwidth = "));
      consolePrintF(logBandWidth);
      consolePrintLn_P(CS_STRING(" bytes/sec"));
      consoleNote_P(CS_STRING("Control latency = "));
      consolePrintFP(controlLatencyAvg / 1e3, 1);
      consolePrintLn_P(CS_STRING(" ms"));

      schedulerReport();
      break;
      
    case c_function:
      if(numParams < 2)
	functionSet(0, NULL);
      else
	functionSet(param[0], &paramText[1][0]);
      break;

    case c_neutral:
      if(numParams > 1 && param[0] >= 0 && param[0] <= MAX_SERVO)
	vpParam.neutral[(int) param[0]] = param[1]/90;
      break;

    case c_scale:
      if(numParams > 0 && param[0] != vpParam.dimension) {
	const float scale = param[0]/vpParam.dimension;
	
	consoleNote_P(CS_STRING("Dimension changed to "));
	consolePrintF(param[0]);
	consolePrint_P(CS_STRING(" (scale by "));
	consolePrintF(scale);
	consolePrintLn(")");

	// Scale CoL by square

	for(i = 0; i < 2; i++) {
	  for(j = 0; j < CoL_degree+1; j++)
	    vpParam.coeff_CoL[i][j] *= sqrf(scale);
	}

	vpParam.dimension = param[0];
      }
      break;

    case c_elevator:
      if(numParams > 0 && param[0] != vpParam.elevDefl) {
	const float value = param[0] / 90.0f, scale = vpParam.elevDefl/value;
	
	consoleNote_P(CS_STRING("Elevator deflection changed to "));
	consolePrintF(param[0]);
	consolePrint_P(CS_STRING(" (scale by "));
	consolePrintF(scale);
	consolePrintLn(")");

	// Scale FF curves

	for(i = 0; i < 2; i++) {
	  for(j = 0; j < FF_degree+1; j++)
	    vpParam.coeff_FF[i][j] *= scale;
	}
	
	vpParam.elevDefl = value;
      }
      break;
      
    case c_ready:
      // Simulate ready for departure
      vpMode.takeOff = true;
      vpStatus.airborne = false;
      break;
      
    case c_launch:
      // Simulate takeoff
      vpMode.takeOff = false;
      vpStatus.airborne = true;
      break;
      
    default:
      consolePrint_P(CS_STRING("Sorry, command not implemented: \""));
      consolePrint(buf);
      consolePrintLn("\"");
      break;
    }

    derivedInvalidate();
  }
}

static void backupParamEntry(const struct Command *e)
{
  int i = 0, j = 0;
  
  consolePrint(e->name);

  for(i = 0; e->var[i]; i++) {
    consolePrint(" ");
    switch(e->varType) {
    case e_string:
      consolePrint((const char*) e->var[i]);
      break;
      
    case e_uint16:
      consolePrintUI(*((uint16_t*) e->var[i]));
      break;
      
    case e_int16:
      consolePrintI(*((int16_t*) e->var[i]));
      break;
      
    case e_int8:
      consolePrintUI8(*((int8_t*) e->var[i]));
      break;
      
    case e_bool:
      consolePrintI(*((bool*) e->var[i]));
      break;
      
    case e_float:
      consolePrintFP(*((float*) e->var[i]), 4);
      break;

    case e_percent:
      consolePrintF(*((float*) e->var[i])*100);
      break;

    case e_angle:
      consolePrintF(*((float*) e->var[i])*RADIAN);
      break;

    case e_angle90:
      consolePrintF(*((float*) e->var[i])*90);
      break;

    case e_map:
      for(j = 0; j < MAX_SERVO; j++) {
	consolePrintI(((int8_t*) e->var[i])[j]);
	consolePrint(" ");
      }
      break;

    case e_nmap:
      for(j = 0; j < MAX_SERVO; j++) {
	consolePrintF(((float*) e->var[i])[j]*90);
	consolePrint(" ");
      }
      break;

    case e_col_curve:
      for(j = 0; j < CoL_degree+1; j++) {
	consolePrintF(((float*) e->var[i])[j]);
	consolePrint(" ");
      }
      break;

    case e_ff_curve:
      for(j = 0; j < FF_degree+1; j++) {
	consolePrintF(((float*) e->var[i])[j]);
	consolePrint(" ");
      }
      break;

    case e_fuel_curve:
      for(j = 0; j < FuelFlow_degree+1; j++) {
	consolePrintF(((float*) e->var[i])[j]);
	consolePrint(" ");
      }
      break;
    }
  }

  consolePrintLn("");
} 

void backupParams()
{
  datagramTxStart(DG_PARAMS);
  datagramTxOut((const uint8_t*) vpParam.name, strlen(vpParam.name));
  datagramTxEnd();
  
  consoleNoteLn_P(CS_STRING("Param backup"));
  consoleNoteLn("");

  consoleNote_P(CS_STRING("MODEL "));
  consolePrintUI(nvState.model);
  consolePrint_P(CS_STRING(" "));
  consolePrintLn(vpParam.name);
  consoleNoteLn("");
  consolePrintLn("");

  //  consolePrint("model ");
  //  consolePrintLn(nvState.model);

  int i = 0;
  
  while(1) {
    struct Command cache;
    CS_MEMCPY(&cache, &commands[i++], sizeof(cache));
    if(cache.token == c_invalid)
      break;
    if(cache.var[0])
      backupParamEntry(&cache);    
  }

  consolePrintLn_P(CS_STRING("store"));

  datagramTxStart(DG_PARAMS);
  datagramTxEnd();  
}

