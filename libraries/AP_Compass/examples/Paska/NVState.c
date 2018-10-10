#include <string.h>
#include "NVState.h"
#include "M24XX.h"
#include "Math.h"
#include "DSP.h"
#include "Console.h"
#include "CRC16.h"
#include "Datagram.h"
#include "Objects.h"

// NV store layout

struct NVStateRecord nvState;
struct ParamRecord vpParam;
struct DerivedParams vpDerived;

#define stateOffset 0U
#define paramOffset nvState.paramPartition
#define dataOffset nvState.dataPartition

const struct ParamRecord paramDefaults = {
  .crc = 0,
  .name = "Invalid",
  .i2c_clkDiv = 12,
  .i2c_5048B = 0x40, .i2c_24L256 = 0x50, 
  .alphaRef = 0,
  .aileNeutral = 0, .aile2Neutral = 0, .aileDefl = -45.0/90,
  .elevNeutral = 0, .elevDefl = 45.0/90,
  .flapNeutral = 0, .flap2Neutral = 0, .flapDefl = 45.0/90,
  .rudderNeutral = 0, .rudderDefl = 45.0/90,
  .steerNeutral = 0, .steerDefl = 45.0/90, .steerPark = 0,
  .brakeNeutral = 0, .brakeDefl = 45.0/90,
  .canardNeutral = 0, .canardDefl = 45.0/90,
  .vertNeutral = 0, .vertDefl = 45.0/90,
  .horizNeutral = 0, .horizDefl = 45.0/90,
  .functionMap = {0},
  .alphaMax = { 12.0/RADIAN, 12.0/RADIAN },
  .i_Ku_C = 100, .i_Tu = 0.25, .o_P = 0.3, 
  .s_Ku_C = 400, .s_Tu = 0.25, 
  .r_Mix = 0.1,
  .at_Ku = 1, .at_Tu = 2.0,
  .cc_Ku = 3, .cc_Tu = 1.5,
  .coeff_FF = {{0, 0, 0}, {0, 0, 0}},
  .t_Mix = 0.0, .t_Expo = 1.0,
  .maxPitch = 45/RADIAN,
  .roll_C = 0.1,
  .coeff_CoL = { { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }},
  .servoRate = 60/0.09,
  .takeoffTrim = 0.25,
  .weightDry = 1,
  .fuel = 0.5,
  .thrust = 0,
  .thresholdMargin = 0.15,
  .pushMargin = 1.0/RADIAN,
  .yd_C = 5,
  .offset = -0.4/RADIAN,
  .flaperon = 0,
  .virtualOnly = true,
  .haveGear = true,
  .wowCalibrated = false,
  .sensorOrient = false,
  .expo = 0.8,
  .floor = -1,
  .flare = 0.5
};

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
  }
  
  bool isGood = true;
  
  if(paramRecordCrc(&vpParam) != vpParam.crc) {
    if(verbose)
      consolePrintLn_P(CS_STRING(" CORRUPT, using defaults")); 
    vpParam = paramDefaults;
    isGood = false;
  } else if(verbose)
    consolePrintLn_P(CS_STRING(" OK"));

  if(verbose)
    printParams();
  else
    deriveParams();
  
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
  if(!m24xxRead(stateOffset, (uint8_t*) &nvState, sizeof(nvState)))
    return false;
  
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
  int i = 0, j = 0;
  
  deriveParams();
    
  consoleNote_P(CS_STRING("  NAME \""));
  consolePrint(vpParam.name);
  consolePrintLn_P(CS_STRING("\""));
  
  consoleNote_P(CS_STRING("  Weight(dry) = "));
  consolePrintFP(vpDerived.totalMass, 3);
  consolePrint_P(CS_STRING("("));
  consolePrintFP(vpParam.weightDry, 3);
  consolePrint_P(CS_STRING(") kg  thrust = "));
  consolePrintFP(vpParam.thrust, 3);
  consolePrint_P(CS_STRING(" kg ("));
  consolePrintFP(vpParam.thrust/vpDerived.totalMass*100, 0);
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
  for(i = 0; i < FF_degree+1; i++) {
    if(i > 0)
      consolePrint_P(CS_STRING(" + "));
    consolePrintFP(vpDerived.coeff_FF[i], 4);
    consolePrint_P(CS_STRING(" x^"));
    consolePrintI(i);
  }
  consolePrint_P(CS_STRING("  (eff alpha range = "));
  consolePrintF(alphaPredict(-1.0)*RADIAN);
  consolePrint_P(CS_STRING(" ... "));
  consolePrintF(alphaPredict(1.0)*RADIAN);
  consolePrintLn_P(CS_STRING(")"));
  if(vpDerived.haveFlaps) {
    consoleNoteLn_P(CS_STRING("    Feedforward, clean vs full flaps"));
    for(j = 0; j < 2; j++) {
      consoleNote_P(CS_STRING("      "));
      for(i = 0; i < FF_degree+1; i++) {
	if(i > 0)
	  consolePrint_P(CS_STRING(" + "));
	consolePrintFP(vpParam.coeff_FF[j][i], 4);
	consolePrint_P(CS_STRING(" x^"));
	consolePrintI(i);
      }
      consoleNL();
    }
  }
  consoleNote_P(CS_STRING("  Throttle-elev mix comp'n P*IAS^2 (expo) = "));
  consolePrintFP(vpParam.t_Mix, 5);
  consolePrint_P(CS_STRING(" ("));
  consolePrintFP(vpParam.t_Expo, 5);
  consolePrintLn_P(CS_STRING(")"));
  consoleNoteLn_P(CS_STRING("  Stabilizer"));
  consoleNote_P(CS_STRING("    Ku*IAS^1.5 = "));
  consolePrintFP(vpParam.s_Ku_C, 4);
  consolePrint_P(CS_STRING(" Tu = "));
  consolePrintLnFP(vpParam.s_Tu, 4);
  consoleNoteLn_P(CS_STRING("  Yaw damper"));
  consoleNote_P(CS_STRING("    P*IAS^2 = "));
  consolePrintLnFP(vpParam.yd_C, 4);
  consoleNoteLn_P(CS_STRING("  Autothrottle"));
  consoleNote_P(CS_STRING("    Ku = "));
  consolePrintFP(vpParam.at_Ku, 4);
  consolePrint_P(CS_STRING(" Tu = "));
  consolePrintLnFP(vpParam.at_Tu, 4);
  consoleNoteLn_P(CS_STRING("  Auto cruise"));
  consoleNote_P(CS_STRING("    Ku = "));
  consolePrintFP(vpParam.cc_Ku, 4);
  consolePrint_P(CS_STRING(" Tu = "));
  consolePrintLnFP(vpParam.cc_Tu, 4);
  consoleNote_P(CS_STRING("  Climb pitch(max) = "));
  consolePrintFP(vpParam.maxPitch*RADIAN, 2);
  consolePrint_P(CS_STRING("  alt(floor) = "));
  consolePrintLnF(vpParam.floor);
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
  for(i = 0; i < CoL_degree+1; i++) {
    if(i > 0)
      consolePrint_P(CS_STRING(" + "));
    consolePrintFP(vpDerived.coeff_CoL[i], 4);
    consolePrint_P(CS_STRING(" x^"));
    consolePrintI(i);
  }
  consolePrint_P(CS_STRING(" (max = "));
  consolePrintFP(vpDerived.maxCoeffOfLift, 4);
  consolePrintLn(")");
  if(vpDerived.haveFlaps) {
    consoleNoteLn_P(CS_STRING("    CoL clean vs full flaps"));
    for(j = 0; j < 2; j++) {
      consoleNote_P(CS_STRING("      "));
      for(i = 0; i < CoL_degree+1; i++) {
	if(i > 0)
	  consolePrint_P(CS_STRING(" + "));
	consolePrintFP(vpParam.coeff_CoL[j][i], 4);
	consolePrint_P(CS_STRING(" x^"));
	consolePrintI(i);
      }
      consoleNL();
    }
  }
  
  consoleNote_P(CS_STRING("  Flare power = "));
  consolePrintLnFP(vpParam.flare, 3);
  consoleNote_P(CS_STRING("  Roll rate K (expo) = "));
  consolePrintFP(vpParam.roll_C, 3);
  consolePrint_P(CS_STRING(" ("));
  consolePrintFP(vpParam.expo, 3);
  consolePrintLn(")");
  consoleNoteLn_P(CS_STRING("  Elevator"));
  consoleNote_P(CS_STRING("    deflection = "));
  consolePrintF(vpParam.elevDefl*90);
  consolePrint_P(CS_STRING(" neutral = "));
  consolePrintF(vpParam.elevNeutral*90);
  consolePrint_P(CS_STRING(" trim%(takeoff) = "));
  consolePrintLnF(vpParam.takeoffTrim*100);
  consoleNoteLn_P(CS_STRING("  Aileron"));
  consoleNote_P(CS_STRING("    deflection = "));
  consolePrintF(vpParam.aileDefl*90);
  consolePrint_P(CS_STRING(" neutral = "));
  consolePrintF(vpParam.aileNeutral*90);
  consolePrint_P(CS_STRING(" ("));
  consolePrintF(vpParam.aile2Neutral*90);
  consolePrintLn_P(CS_STRING(")"));
  consoleNoteLn_P(CS_STRING("  Canard"));
  consoleNote_P(CS_STRING("    deflection = "));
  consolePrintF(vpParam.canardDefl*90);
  consolePrint_P(CS_STRING(" neutral = "));
  consolePrintLnF(vpParam.canardNeutral*90);
  consoleNoteLn_P(CS_STRING("  Vector "));
  consoleNote_P(CS_STRING("    vertical deflection = "));
  consolePrintF(vpParam.vertDefl*90);
  consolePrint_P(CS_STRING(" neutral = "));
  consolePrintLnF(vpParam.vertNeutral*90);
  consoleNote_P(CS_STRING("    horizontal deflection = "));
  consolePrintF(vpParam.horizDefl*90);
  consolePrint_P(CS_STRING(" neutral = "));
  consolePrintLnF(vpParam.horizNeutral*90);
  consoleNoteLn_P(CS_STRING("  Rudder"));
  consoleNote_P(CS_STRING("    deflection = "));
  consolePrintF(vpParam.rudderDefl*90);
  consolePrint_P(CS_STRING(" neutral = "));
  consolePrintF(vpParam.rudderNeutral*90);
  consolePrint_P(CS_STRING(" aile mix = "));
  consolePrintLnF(vpParam.r_Mix);
  consoleNoteLn_P(CS_STRING("  Steering"));
  consoleNote_P(CS_STRING("    deflection = "));
  consolePrintF(vpParam.steerDefl*90);
  consolePrint_P(CS_STRING(" neutral = "));
  consolePrintLnF(vpParam.steerNeutral*90);
  consoleNoteLn_P(CS_STRING("  Flap"));
  consoleNote_P(CS_STRING("    defl = "));
  consolePrintF(vpParam.flapDefl*90);
  consolePrint_P(CS_STRING(" neutral = "));
  consolePrintF(vpParam.flapNeutral*90);
  consolePrint_P(CS_STRING(" ("));
  consolePrintF(vpParam.flap2Neutral*90);
  consolePrintLn_P(CS_STRING(")"));
  consoleNote_P(CS_STRING("  Servo rate = "));
  consolePrintLnF(vpParam.servoRate);
  if(vpParam.flaperon)
    consoleNoteLn_P(CS_STRING("  Flaperon ENABLED"));

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

void deriveParams()
{
  int i = 0;
  
  // Do we have rectracts and/or flaps?

  vpDerived.haveRetracts = false;
  vpDerived.haveFlaps = vpParam.flaperon;

  if(vpParam.haveGear) {
    for(i = 0; i < MAX_SERVO; i++)
      if(vpParam.functionMap[i] == fn_gear) {
	vpDerived.haveRetracts = true;
	break;
      }
  }

  for(i = 0; i < MAX_SERVO; i++)
    if(vpParam.functionMap[i] == fn_leftflap
       || vpParam.functionMap[i] == fn_rightflap) {
      vpDerived.haveFlaps = true;
      break;
    }

  // Total mass

  vpDerived.totalMass = vpParam.weightDry + vpParam.fuel;

  // Max alpha and curve interpolation

  vpDerived.assumedFlap = vpOutput.flap;

  const float expo_c = 0.8;
  
  const float effFlap
    // = vpDerived.haveFlaps ? powf(vpOutput.flap, vpParam.expo) : 0;
    // = vpDerived.haveFlaps ? vpOutput.flap : 0;
    = vpDerived.haveFlaps ? powf(vpOutput.flap, expo_c) : 0;
  
  vpDerived.maxAlpha = interpolate(effFlap, vpParam.alphaMax);
  
  interpolate_CoL(effFlap, vpDerived.coeff_CoL);
  interpolate_FF(effFlap, vpDerived.coeff_FF);
  
  // Max CoL

  vpDerived.maxCoeffOfLift = coeffOfLift(vpDerived.maxAlpha);
  vpDerived.maxCoeffOfLiftClean = coeffOfLiftClean(vpParam.alphaMax[0]);
  
  // Zero lift alpha
  
  vpDerived.zeroLiftAlpha = coeffOfLiftInverse(0);

  // Stall IAS
  
  vpDerived.minimumDynP = G * vpDerived.totalMass / vpDerived.maxCoeffOfLift;
  vpDerived.minimumIAS = dynamicPressureInverse(vpDerived.minimumDynP);
  
  //
  // Effective alpha limits
  //
  
  vpDerived.thresholdAlpha =
    coeffOfLiftInverse(vpDerived.maxCoeffOfLift/square(1 + vpParam.thresholdMargin));
  vpDerived.shakerAlpha =
    coeffOfLiftInverse(vpDerived.maxCoeffOfLift/square(1 + vpParam.thresholdMargin/2));
  vpDerived.pusherAlpha = vpDerived.maxAlpha - fmax(vpParam.pushMargin, 0);
  
  vpDerived.stallAlpha = vpDerived.maxAlpha - fmin(vpParam.pushMargin, 0);
  
  //
  // Feedforward apex
  //

  if(vpDerived.coeff_FF[2] != 0.0) {
    vpDerived.apexAlpha = -vpDerived.coeff_FF[1] / (2*vpDerived.coeff_FF[2]);
    vpDerived.apexElev =
      polynomial(FF_degree, vpDerived.apexAlpha, vpDerived.coeff_FF);
  } else {
    vpDerived.apexAlpha = vpDerived.apexElev = 0.0;
  }
}

