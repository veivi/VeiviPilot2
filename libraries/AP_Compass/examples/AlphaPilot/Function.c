#include <string.h>
#include "Function.h"
#include "Objects.h"
#include "NVState.h"
#include "Console.h"
#include "Math.h"

//
// Actuator functions
//

int16_t aileronFn()
{
  return vpParam.aileDefl*vpOutput.aile;
}

int16_t elevatorFn()
{
  return vpParam.elevDefl*vpOutput.elev;
}

int16_t rudderFn()
{
  return vpParam.rudderDefl*vpOutput.rudder;
}

int16_t flapFn()
{
  return vpParam.flapDefl*vpOutput.flap;
}

int16_t elevon1Fn()
{
  return aileronFn() - elevatorFn();
}

int16_t elevon2Fn()
{
  return aileronFn() + elevatorFn();
}

int16_t flaperon1Fn()
{
  return aileronFn() + flapFn();
}

int16_t flaperon2Fn()
{
  return aileronFn() - flapFn();
}

int16_t tail1Fn()
{
  return elevatorFn() + rudderFn();
}

int16_t tail2Fn()
{
  return elevatorFn() - rudderFn();
}

int16_t canard1Fn()
{
  return vpOutput.canard*(500.0f/(PI_F/2)) + vpParam.canardDefl*vpOutput.elev;
}

int16_t canard2Fn()
{
  return -canard1Fn();
}

int16_t thrustVertFn()
{
  return vpParam.vertDefl*vpOutput.thrustVert;
}

int16_t thrustHorizFn()
{
  return vpParam.horizDefl*vpOutput.thrustHoriz;
}

int16_t steeringFn()
{
  if(vpDerived.haveRetracts && vpControl.gearSel)
    return vpParam.steerPark;
  else
    return vpParam.steerDefl*(vpOutput.steer + vpParam.steerTrim);
}

int16_t lightFn()
{
  int16_t value = -0.10f*500;

  if(vpMode.takeOff || vpStatus.airborne) {
    if(vpControl.gearState == gs_down)
      value = 0.50f*500;
    else
      value = 0.12f*500;
  }

  return value;
}

int16_t gearFn()
{
  int16_t value = 0;

  if(!vpMode.gearSelected)
    return 0;
  
  switch(vpControl.gearState) {
  case gs_goingdown_close:
  case gs_down:
  case gs_goingup_open:
    value = 400;
    break;
    
  case gs_goingup:
  case gs_goingup_close:
  case gs_up:
  case gs_goingdown_open:
    value = -400;
    break;
    
  case gs_goingdown:
    value = vpControl.pwmCount < vpParam.gearSpeed ? 400 : 0;
    break;
  }
  
  return value;
}

int16_t doorFn()
{
  int8_t value = 0;

  switch(vpControl.gearState) {
  case gs_goingdown_open:
  case gs_goingdown:
  case gs_goingup_open:
  case gs_goingup:
    value = -1;
    break;

  default:
    value = 1;
    break;
  }

  return (float) value;
}

#define BRAKE_PWM_HZ    7
#define BRAKE_THRESHOLD 0.2
#define BRAKE_BOOST     4

int16_t brakeFn()
{
  // Brake PWM

  /*
  static int8_t count;

  if(count < controlFreq/BRAKE_PWM_HZ - 1)
    count++;
  else
    count = 0;

  if(vpOutput.brake > 1.0f)
    return 1.0f;
  else if(vpOutput.brake < BRAKE_THRESHOLD
     || count > vpOutput.brake*controlFreq/BRAKE_PWM_HZ-1)
    return 0.0f;

  return
    vpParam.brakeDefl * (1.0f - 1.0f/BRAKE_BOOST + vpOutput.brake/BRAKE_BOOST);

  */

  return vpParam.brakeDefl*vpOutput.brake;
}

int16_t throttleFn()
{
  return 800*turbineOutput(&engine);
}

struct FnDescriptor {
  const char *name;
  int16_t (*code)(void);
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
  [fn_thrusthoriz] = { "horiz", thrustHorizFn },
  [fn_light] = { "light", lightFn },
  [fn_door] = { "door", doorFn }
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
      consolePrintFP(vpParam.neutral[i]*(90.0f/500), 1);
      consoleTab(30);
      consolePrintI(vpParam.neutral[i]);
      consolePrintLn_P(CS_STRING(" us"));
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

bool functionInvoke(int8_t fn, int16_t *result)
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
