#include <string.h>
#include "NVState.h"
#include "M24XX.h"
#include "Math.h"
#include "DSP.h"
#include "Console.h"
#include "CRC16.h"
#include "Datagram.h"
#include "Objects.h"
#include "Function.h"

// NV store layout

struct NVStateRecord nvState;
struct ParamRecord vpParam;
struct DerivedParams vpDerived;

#define stateOffset 0U
#define paramOffset nvState.paramPartition
#define dataOffset nvState.dataPartition

const struct ParamRecord paramDefaults = {
  .crc = 0,
  .version = PARAM_VERSION,
  .name = "Invalid",
  .dimension = 1.0f,
  .shakerMargin = 0.5 };

const struct NVStateRecord stateDefaults = {
  .crc = 0,
  .paramPartition = 128,
  .dataPartition = 2048,
  .logPartition = 4096,
  .logStamp = 400,
  .model = 0 };

static uint16_t crc16OfRecord(uint16_t initial, const uint8_t *record, int size)
{
  return crc16(initial, &record[sizeof(uint16_t)], size - sizeof(uint16_t));
}

static uint16_t paramRecordCrc(struct ParamRecord *record)
{
  return crc16OfRecord(0xFFFF, (uint8_t*) record, sizeof(*record));
}

static uint16_t stateRecordCrc(struct NVStateRecord *record)
{
  return crc16OfRecord(0xFFFF, (uint8_t*) record, sizeof(*record));
}

void defaultParams(void)
{
  vpParam = paramDefaults;
}

void defaultState(void)
{
  nvState = stateDefaults;
}

int maxModels(void)
{
  return (nvState.dataPartition - nvState.paramPartition) / sizeof(vpParam);
}

bool setModel(int model, bool verbose)
{
  if(model < 0)
    model = 0;
  else if(model > maxModels() - 1)
    model = maxModels() - 1;
  
  nvState.model = model;
  m24xxRead(paramOffset + sizeof(vpParam)*model,
	    (uint8_t*) &vpParam, sizeof(vpParam));

  if(verbose) {
    consoleNote_P(CS_STRING("MODEL "));
    consolePrintLnUI(model);
    consoleNote_P(CS_STRING("  Model record CRC = "));
    consolePrintUI(vpParam.crc);
    consolePrint_P(CS_STRING(" Ver = "));
    consolePrintUI(vpParam.version);
  }
  
  bool isGood = true;
  
  if(paramRecordCrc(&vpParam) != vpParam.crc
     || vpParam.version != PARAM_VERSION) {
    if(verbose)
      consolePrintLn_P(CS_STRING(" CORRUPT, using defaults"));
    defaultParams();
    isGood = false;
  } else if(verbose)
    consolePrintLn_P(CS_STRING(" OK"));

  vpDerived.valid = false;
  
  if(isGood && verbose)
    printParams();
  
  return isGood;
}
  
void storeParams(void)
{
  vpParam.crc = paramRecordCrc(&vpParam);
  consoleNote_P(CS_STRING("Model record CRC = "));
  consolePrintLnUI(vpParam.crc);
  m24xxWrite(paramOffset + sizeof(vpParam)*nvState.model,
  	     (const uint8_t*) &vpParam, sizeof(vpParam));
  m24xxFlush();
  consoleNoteLn_P(CS_STRING("  Stored"));
}

void deleteModel(int model)
{
  m24xxWrite(paramOffset + sizeof(struct ParamRecord)*model,
  	     NULL, sizeof(struct ParamRecord));
  m24xxFlush();
}

void readData(uint8_t *data, int size)
{
  m24xxRead(dataOffset, data, size);
}

void storeData(const uint8_t *data, int size)
{
  m24xxWrite(dataOffset, data, size);
  m24xxFlush();
}

bool readNVState(void)
{
  if(!m24xxRead(stateOffset, (uint8_t*) &nvState, sizeof(nvState))) {
    consoleNote_P(CS_STRING("NV State read FAILED, using defaults."));
    defaultState();
    return false;
  }
  
  consoleNote_P(CS_STRING("  State record CRC = "));
  consolePrintUI(nvState.crc);
    
  if(nvState.crc != stateRecordCrc(&nvState)) {
    consolePrintLn_P(CS_STRING(" CORRUPT, using defaults")); 
    defaultState();
  } else
    consolePrintLn_P(CS_STRING(" OK"));

  return true;
}

void storeNVState(void)
{
  if(sizeof(nvState) < paramOffset - stateOffset) {
    nvState.crc = stateRecordCrc(&nvState);
    m24xxWrite(stateOffset, (const uint8_t*) &nvState, sizeof(nvState));
    m24xxFlush();
  } else
    consoleNoteLn_P(CS_STRING("PANIC : State record exceeds partition size"));
}

void printParams()
{
  int j = 0;
  
  derivedValidate();
    
  consoleNote_P(CS_STRING("  NAME \""));
  consolePrint(vpParam.name);
  consolePrintLn_P(CS_STRING("\""));
  
  consoleNote_P(CS_STRING("    Dimension = "));
  consolePrintLnFP(vpParam.dimension, 3);
  consoleNote_P(CS_STRING("  Weight (dry, fuel, batt) = "));
  consolePrintFP(vpDerived.takeoffMass, 3);
  consolePrint_P(CS_STRING(" ("));
  consolePrintFP(vpParam.weightDry, 3);
  consolePrint_P(CS_STRING(", "));
  consolePrintFP(vpParam.fuel, 3);
  consolePrint_P(CS_STRING(", "));
  consolePrintFP(vpParam.battery, 3);
  consolePrintLn_P(CS_STRING(") kg"));
  consoleNote_P(CS_STRING("    Thrust = "));
  consolePrintFP(vpParam.thrust, 3);
  consolePrint_P(CS_STRING(" kg ("));
  consolePrintFP(vpParam.thrust/vpDerived.takeoffMass*100, 0);
  consolePrintLn_P(CS_STRING("% of weight)"));
  consoleNote_P(CS_STRING("  AS5048B ref = "));
  consolePrintLnUI(vpParam.alphaRef);
  consoleNoteLn_P(CS_STRING("  Alpha Hold"));
  consoleNote_P(CS_STRING("    Inner Ku*IAS^1.5 = "));
  consolePrintFP(vpParam.i_Ku_C, 4);
  consolePrint_P(CS_STRING(" Tu = "));
  consolePrintFP(vpParam.i_Tu, 4);
  consolePrint_P(CS_STRING(" Outer P = "));
  consolePrintLnFP(vpParam.o_P, 4);
  consoleNoteLn_P(CS_STRING("  Elev feedforward A+Bx+Cx^2"));
  consoleNote_P(CS_STRING("    "));
  consolePrintPoly(FF_degree, vpDerived.coeff_FF, 4);
  consolePrint_P(CS_STRING("  (eff alpha range = "));
  consolePrintF(alphaPredict(-1.0)*RADIAN);
  consolePrint_P(CS_STRING(" ... "));
  consolePrintF(alphaPredict(1.0)*RADIAN);
  consolePrintLn_P(CS_STRING(")"));
  if(vpDerived.haveFlaps) {
    consoleNoteLn_P(CS_STRING("    Feedforward, clean vs full flaps"));
    for(j = 0; j < 2; j++) {
      consoleNote_P(CS_STRING("      "));
      consolePrintPoly(FF_degree, vpParam.coeff_FF[j], 4);
      consoleNL();
    }
  }
  consoleNote_P(CS_STRING("  Throttle trim compensation P*IAS^2 (expo) = "));
  consolePrintFP(vpParam.t_Mix, 5);
  consolePrint_P(CS_STRING(" ("));
  consolePrintFP(vpParam.t_Expo, 5);
  consolePrintLn_P(CS_STRING(")"));
  consoleNoteLn_P(CS_STRING("  Stabilizer"));
  consoleNote_P(CS_STRING("    Ku*IAS^1.5 = "));
  consolePrintFP(vpParam.s_Ku_C, 4);
  consolePrint_P(CS_STRING(" Tu = "));
  consolePrintLnFP(vpParam.s_Tu, 4);
  consoleNoteLn_P(CS_STRING("  Auto-rudder"));
  consoleNote_P(CS_STRING("    Ku*IAS^1.5 = "));
  consolePrintFP(vpParam.r_Ku_C, 4);
  consolePrint_P(CS_STRING(" Tu = "));
  consolePrintLnFP(vpParam.r_Tu, 4);
  consoleNoteLn_P(CS_STRING("  Yaw damper"));
  consoleNote_P(CS_STRING("    P*IAS^2 = "));
  consolePrintLnFP(vpParam.yd_C, 4);
  consoleNoteLn_P(CS_STRING("  Turbine"));
  consoleNote_P(CS_STRING("    idle = "));
  consolePrintFP(vpParam.idle, 2);
  consolePrint_P(CS_STRING(" lag = "));
  consolePrintLnFP(vpParam.lag, 2);
  consoleNote_P(CS_STRING("    Fuel flow = "));
  consolePrintPoly(FuelFlow_degree, vpParam.coeff_Flow, 1);
  consolePrint_P(CS_STRING(" ("));
  consolePrintFP(polynomial(FuelFlow_degree, 0, vpParam.coeff_Flow), 1);
  consolePrint_P(CS_STRING(" ... "));
  consolePrintFP(polynomial(FuelFlow_degree, 1, vpParam.coeff_Flow), 1);
  consolePrintLn_P(CS_STRING(" g/min)"));
  consoleNote_P(CS_STRING("  Climb pitch(max) = "));
  consolePrintLnFP(vpParam.maxPitch*RADIAN, 2);
  consoleNote_P(CS_STRING("  Max alpha (clean, full flaps) = "));
  consolePrintF(vpDerived.maxAlpha*RADIAN);
  
  if(vpDerived.haveFlaps) {
    consolePrint(" (");
    consolePrintF(vpParam.alphaMax[0]*RADIAN);
    consolePrint(", ");
    consolePrintF(vpParam.alphaMax[1]*RADIAN);
    consolePrint(")");
  }
  consoleNL();
  consoleNote_P(CS_STRING("  Alpha range = "));
  consolePrintF(vpDerived.zeroLiftAlpha*RADIAN);
  consolePrint_P(CS_STRING(" ... "));
  consolePrintF(vpDerived.maxAlpha*RADIAN);
  consolePrint_P(CS_STRING(", minimum IAS = "));
  consolePrintLnF(vpDerived.minimumIAS);
  consoleNote_P(CS_STRING("  Threshold margin(%) = "));
  consolePrintF(vpParam.thresholdMargin*100);
  consolePrint_P(CS_STRING(" Shaker (%thres) = "));
  consolePrintF(vpParam.shakerMargin*100);
  consolePrint_P(CS_STRING(" Pusher(deg) = "));
  consolePrintLnF(vpParam.pushMargin*RADIAN);
  consoleNote_P(CS_STRING("    Derived alpha(threshold, shake, push, stall) = "));
  consolePrintF(vpDerived.thresholdAlpha*RADIAN);
  consolePrint_P(CS_STRING(", "));
  consolePrintF(vpDerived.shakerAlpha*RADIAN);
  consolePrint_P(CS_STRING(", "));
  consolePrintF(vpDerived.pusherAlpha*RADIAN);
  consolePrint_P(CS_STRING(", "));
  consolePrintLnF(vpDerived.stallAlpha*RADIAN);
  consoleNoteLn_P(CS_STRING("  Coeff of lift"));
  consoleNote_P(CS_STRING("    "));
  consolePrintPoly(CoL_degree, vpDerived.coeff_CoL, 4);
  consolePrint_P(CS_STRING(" (max = "));
  consolePrintFP(vpDerived.maxCoeffOfLift, 4);
  consolePrintLn(")");
  if(vpDerived.haveFlaps) {
    consoleNoteLn_P(CS_STRING("    CoL clean vs full flaps"));
    for(j = 0; j < 2; j++) {
      consoleNote_P(CS_STRING("      "));
      consolePrintPoly(CoL_degree, vpParam.coeff_CoL[j], 4);
      consoleNL();
    }
  }
  
  consoleNote_P(CS_STRING("  Flare power = "));
  consolePrintLnFP(vpParam.flare, 3);
  consoleNote_P(CS_STRING("  Roll rate K (expo) = "));
  consolePrintFP(vpParam.roll_C, 3);
  consolePrint_P(CS_STRING(" ("));
  consolePrintFP(vpParam.roll_Expo, 3);
  consolePrintLn(")");
  consoleNoteLn_P(CS_STRING("  CONTROL SURFACE DEFLECTIONS"));
  consoleNote_P(CS_STRING("    Elevator    "));
  consolePrintF(vpParam.elevDefl*90);
  consoleTab(30);
  consolePrint_P(CS_STRING("trim%(takeoff) = "));
  consolePrintLnF(vpParam.takeoffTrim*100);
  consoleNote_P(CS_STRING("    Aileron     "));
  consolePrintF(vpParam.aileDefl*90);
  consoleTab(30);
  consolePrint_P(CS_STRING("servo rate = "));
  consolePrintLnF(vpParam.servoRate);
  consoleNote_P(CS_STRING("    Canard      "));
  consolePrintF(vpParam.canardDefl*90);
  consolePrint_P(CS_STRING(" (gain "));
  consolePrintF(vpParam.canardGain);
  consolePrintLn_P(CS_STRING(")")); 
  consoleNote_P(CS_STRING("    Vector(v,h) "));
  consolePrintF(vpParam.vertDefl*90);
  consolePrint(", ");
  consolePrintLnF(vpParam.horizDefl*90);
  consoleNote_P(CS_STRING("    Rudder      "));
  consolePrintF(vpParam.rudderDefl*90);
  consoleTab(30);
  consolePrint_P(CS_STRING("aile mix = "));
  consolePrintLnF(vpParam.r_Mix);
  consoleNote_P(CS_STRING("    Nose wheel  "));
  consolePrintF(vpParam.steerDefl*90);
  consoleTab(30);
  consolePrint_P(CS_STRING("trim = "));
  consolePrintLnF(vpParam.steerTrim*90);
  consoleNote_P(CS_STRING("    Flap        "));
  consolePrintLnF(vpParam.flapDefl*90);
  consoleNote_P(CS_STRING("    Brake       "));
  consolePrintLnF(vpParam.brakeDefl*90);

  consoleNote_P(CS_STRING("  We"));
  if(!vpParam.haveGear)
    consolePrint_P(CS_STRING(" DO NOT"));  
  consolePrint_P(CS_STRING(" have"));

  if(vpParam.haveGear) {
    if(vpDerived.haveRetracts)
      consolePrint_P(CS_STRING(" RETRACTABLE"));
    else
      consolePrint_P(CS_STRING(" FIXED"));
  }
  
  consolePrintLn_P(CS_STRING(" landing GEAR"));
  if(vpParam.haveGear) {
    consoleNote_P(CS_STRING("    Weight on wheels"));
    if(!vpParam.wowCalibrated)
      consolePrint_P(CS_STRING(" NOT"));  
    consolePrintLn_P(CS_STRING(" CALIBRATED"));
  } else
    vpParam.wowCalibrated = false;
}

static float interpolate(float c, const float v[])
{
  return mixValue(c, v[0], v[1]);
}

static void interpolate_vector(int d, float c, float r[], const float a[], const float b[])
{
  int i = 0;
  
  for(i = 0; i < d; i++)
    r[i] = mixValue(c, a[i], b[i]);
}   

static void interpolate_CoL(float c, float r[])
{
  interpolate_vector(CoL_degree+1, c, r, vpParam.coeff_CoL[0], vpParam.coeff_CoL[1]);
}

static void interpolate_FF(float c, float r[])
{
  interpolate_vector(FF_degree+1, c, r, vpParam.coeff_FF[0], vpParam.coeff_FF[1]);
}

void derivedInvalidate()
{
  vpDerived.valid = false;
}

void derivedValidate()
{
  int i = 0;
  
  if(vpDerived.valid)
    return;
  
  vpDerived.valid = true;

  // Takeoff mass

  vpDerived.takeoffMass = vpParam.weightDry + vpParam.fuel + vpParam.battery;
  
  // Do we have rectracts and/or flaps?

  vpDerived.haveRetracts = vpDerived.haveFlaps = false;

  if(vpParam.haveGear) {
    for(i = 0; i < MAX_SERVO; i++)
      if(ABS(vpParam.functionMap[i]) == fn_gear) {
	vpDerived.haveRetracts = true;
	break;
      }
  }

  for(i = 0; i < MAX_SERVO; i++)
    if(vpParam.functionMap[i] == fn_flap
       || vpParam.functionMap[i] == fn_flaperon1
       || vpParam.functionMap[i] == fn_flaperon2) {
      vpDerived.haveFlaps = true;
      break;
    }

  // Store the assumed values for flaps and mass
  
  vpDerived.assumedFlap = vpOutput.flap;
  vpDerived.assumedMass = vpStatus.mass;

  // Max alpha and curve interpolation

  const float expo_c = 0.8;
  
  const float effFlap
    // = vpDerived.haveFlaps ? powf(vpOutput.flap, vpParam.expo) : 0;
    // = vpDerived.haveFlaps ? vpOutput.flap : 0;
    = vpDerived.haveFlaps ? powf(vpDerived.assumedFlap, expo_c) : 0;
  
  vpDerived.maxAlpha = interpolate(effFlap, vpParam.alphaMax);
  
  interpolate_CoL(effFlap, vpDerived.coeff_CoL);
  interpolate_FF(effFlap, vpDerived.coeff_FF);
  
  // Max CoL

  vpDerived.maxCoeffOfLift = coeffOfLift(vpDerived.maxAlpha);
  vpDerived.maxCoeffOfLiftClean = coeffOfLiftClean(vpParam.alphaMax[0]);
  
  // Zero lift alpha
  
  vpDerived.zeroLiftAlpha = coeffOfLiftInverse(0);

  // Stall IAS
  
  vpDerived.minimumDynP = G * vpDerived.assumedMass / vpDerived.maxCoeffOfLift;
  vpDerived.minimumIAS = dynamicPressureInverse(vpDerived.minimumDynP);
  
  //
  // Effective alpha limits
  //
  
  vpDerived.thresholdAlpha =
    coeffOfLiftInverse(vpDerived.maxCoeffOfLift/sqrf(1 + vpParam.thresholdMargin));
  vpDerived.pusherAlpha = vpDerived.maxAlpha - fmaxf(vpParam.pushMargin, 0);
  
  vpDerived.shakerAlpha =
    fminf(coeffOfLiftInverse(vpDerived.maxCoeffOfLift/sqrf(1 + vpParam.thresholdMargin*vpParam.shakerMargin)), vpDerived.pusherAlpha);

  vpDerived.stallAlpha = vpDerived.maxAlpha - fminf(vpParam.pushMargin, 0);
  
  //
  // Feedforward apex
  //

  if(vpDerived.coeff_FF[2] != 0.0f) {
    vpDerived.apexAlpha = -vpDerived.coeff_FF[1] / (2*vpDerived.coeff_FF[2]);
    vpDerived.apexElev =
      polynomial(FF_degree, vpDerived.apexAlpha, vpDerived.coeff_FF);
  } else {
    vpDerived.apexAlpha = vpDerived.apexElev = 0.0f;
  }
}

