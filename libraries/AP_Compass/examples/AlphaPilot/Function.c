#include <string.h>
#include "Function.h"
#include "Objects.h"
#include "NVState.h"
#include "Console.h"
#include "Math.h"

//
// Actuator functions
//

float elevatorFn()
{
  return vpParam.elevDefl*vpOutput.elev;
}

float elevon1Fn()
{
  return vpParam.aileDefl*vpOutput.aile - vpParam.elevDefl*vpOutput.elev;
}

float elevon2Fn()
{
  return vpParam.aileDefl*vpOutput.aile + vpParam.elevDefl*vpOutput.elev;
}

float aileronFn()
{
  return vpParam.aileDefl*vpOutput.aile;
}

static float flaperon()
{
  return vpParam.flapDefl*vpOutput.flap;
}

float flaperon1Fn()
{
  return aileronFn() + flaperon();
}

float flaperon2Fn()
{
  return aileronFn() - flaperon();
}

float rudderFn()
{
  return vpParam.rudderDefl*vpOutput.rudder;
}

float tail1Fn()
{
  return vpParam.elevDefl*vpOutput.elev + vpParam.rudderDefl*vpOutput.rudder;
}

float tail2Fn()
{
  return vpParam.elevDefl*vpOutput.elev - vpParam.rudderDefl*vpOutput.rudder;
}

float canard1Fn()
{
  return vpParam.canardDefl*vpOutput.elev;
}

float canard2Fn()
{
  return -canard1Fn();
}

float thrustVertFn()
{
  return vpParam.vertDefl*vpOutput.thrustVert;
}

float thrustHorizFn()
{
  return vpParam.horizDefl*vpOutput.thrustHoriz;
}

float steeringFn()
{
  if(vpDerived.haveRetracts && vpControl.gearSel)
    return vpParam.steerPark;
  else
    return vpParam.steerTrim + vpParam.steerDefl*vpOutput.steer;
}

float flapFn()
{
  return vpParam.flapDefl*vpOutput.flap;
}

float gearFn()
{
  return -RATIO(2/3)*(vpControl.gearSel*2-1);
}

float brakeFn()
{
  return vpParam.brakeDefl*vpOutput.brake;
}

float throttleFn()
{
  return THROTTLE_SIGN*RATIO(2/3)*(2*turbineOutput(&engine) - 1);
}

struct FnDescriptor {
  const char *name;
  float (*code)(void);
};

struct FnDescriptor functionTable[] = {
  [fn_null] = { "-", NULL },
  [fn_aileron] = { "aile", aileronFn },
  [fn_elevator] = { "elev", elevatorFn },
  [fn_rudder] = { "rud", rudderFn },
  [fn_throttle] = { "thro", throttleFn },
  [fn_gear] = { "gear", gearFn },
  [fn_steering] = { "nose", steeringFn },
  [fn_brake] = { "brake", brakeFn },
  [fn_flaperon1] = { "flaperon1", flaperon1Fn },
  [fn_flaperon2] = { "flaperon2", flaperon2Fn },
  [fn_canard1] = { "canard1", canard1Fn },
  [fn_canard2] = { "canard2", canard2Fn },
  [fn_elevon1] = { "elevon1", elevon1Fn },
  [fn_elevon2] = { "elevon2", elevon2Fn },
  [fn_tail1] = { "tail1", tail1Fn },
  [fn_tail2] = { "tail2", tail2Fn },
  [fn_flap] = { "flap", flapFn },
  [fn_thrustvert] = { "vert", thrustVertFn },
  [fn_thrusthoriz] = { "horiz", thrustHorizFn }
};

void functionSet(uint8_t ch, const char *name)
{
  int i = 0;
  
  if(ch > MAX_SERVO-1 || !name) {
    consoleNoteLn_P(CS_STRING("SERVO  FUNCTION  NEUTRAL"));
    consoleNoteLn_P(CS_STRING("------------------------"));

    for(i = 0; i < MAX_SERVO; i++) {
      bool reverse = vpParam.functionMap[i] < 0;
	  
      consoleNote_P(CS_STRING("  "));
      consolePrintI(i);
      consoleTab(10);

      if(reverse)
	consolePrint("-");

      if(ABS(vpParam.functionMap[i]) < fn_invalid)
	consolePrint(functionTable[ABS(vpParam.functionMap[i])].name);
      else
	consolePrint_P(CS_STRING("*invalid*"));
      
      consoleTab(20);
      consolePrintLnFP(vpParam.neutral[i]*90, 1);
    }

    consoleNoteLn("");
    consoleNote_P(CS_STRING("AVAILABLE FUNCTIONS: "));

    for(i = 0; i < fn_invalid; i++) {
      if(i > 0)
	consolePrint(", ");
      consolePrint(functionTable[i].name);
    }

    consoleNL();
    return;
  }
  
  bool reverse = false, success = false;

  if(name[0] == '-' && name[1] != '\0') {
    reverse = true;
    name++;
  }

  for(i = 0; i < sizeof(functionTable)/sizeof(struct FnDescriptor); i++) {
    if(!strcmp(name, functionTable[i].name)) {
      vpParam.functionMap[ch] = reverse ? -i : i;
      success = true;
    }
  }
  
  if(!success) {
    consoleNote_P(CS_STRING("Unindentified function "));
    consolePrintLn(name);
  }
}

bool functionInvoke(int8_t fn, float *result)
{
  function_t i = ABS(fn);
    
  if(i < fn_invalid && functionTable[i].code) {
    *result = functionTable[i].code();
    if(fn < 0)
      *result = -*result;
    return true;
  } else
    return false;
}
