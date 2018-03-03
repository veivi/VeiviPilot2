#ifndef NVSTATE_H
#define NVSTATE_H

#include <stdint.h>
#include <AP_ProgMem/AP_ProgMem.h>

#define NAME_LEN     8
#define MAX_CH       8

// Parameters and non-volatile state

struct ParamRecord {
  uint16_t crc;
  char name[NAME_LEN+1];
  uint8_t i2c_clkDiv;
  uint8_t i2c_5048B, i2c_24L256;
  uint16_t alphaRef;
  float aileNeutral, aile2Neutral, aileDefl;
  float elevNeutral, elevDefl;
  float flapNeutral, flap2Neutral, flapDefl;
  float rudderNeutral, rudderDefl;
  float steerNeutral, steerDefl;
  float brakeNeutral, brakeDefl;
  float canardNeutral, canardDefl;
  float vertNeutral, vertDefl;
  float horizNeutral, horizDefl;
  int8_t servoAile, servoAile2, servoElev, servoRudder, servoFlap, servoFlap2, servoGear, servoBrake, servoSteer, servoThrottle, servoLeft, servoRight, servoVertLeft, servoVertRight, servoHoriz;
  float cL_A, alphaMax;
  float i_Ku_C, i_Tu, o_P;
  float s_Ku_C, s_Tu;
  float r_Mix;
  //  float p_Ku_C, p_Tu;
  float at_Ku, at_Tu;
  float cc_Ku, cc_Tu;
  float ff_A, ff_B, ff_C;
  float t_Mix, t_Expo;
  float maxPitch;
  float cL_apex;
  float roll_C;
  float cL_B, cL_C, cL_D, cL_E;
  float servoRate;
  float takeoffTrim;
  float weightDry, fuel, thrust;
  float thresholdMargin, stallMargin;
  float glideSlope;
  float offset;
  bool elevon, veeTail, flaperon;
  bool virtualOnly;
  bool haveWheels;
  bool wowCalibrated;
  float expo;
  int16_t floor;
  };

struct DerivedParams {
  float totalMass;
  float minimumIAS, minimumDynP, zeroLiftAlpha, maxCoeffOfLift;
  float thresholdAlpha, shakerAlpha, pusherAlpha;
  float apexAlpha, apexElev;
};

struct NVStateRecord {
  uint16_t crc;
  uint16_t paramPartition, dataPartition, logPartition;
  uint16_t logStamp;
  uint16_t model;
  uint16_t testNum;
  int32_t rxCenter[MAX_CH], rxMin[MAX_CH], rxMax[MAX_CH];
};

extern struct ParamRecord vpParam;
extern struct NVStateRecord nvState;
extern struct DerivedParams vpDerived;
extern const prog_char_t *updateDescription;

void defaultParams(void);
bool setModel(int model, bool verbose);
void deleteModel(int model);
void storeParams(void);
void readNVState(void);
void storeNVState(void);
void printParams(void);
void deriveParams();
void backupParams(void);
int maxModels(void);
void readData(uint8_t *data, int size);
void storeData(const uint8_t *data, int size);
  
#endif
