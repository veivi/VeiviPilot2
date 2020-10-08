#ifndef NVSTATE_H
#define NVSTATE_H

#include <stdint.h>
#include <stdbool.h>
#include "AS5048B.h"

#define NAME_LEN     8
#define MAX_CH       12
#define MAX_SERVO    11
#define MAX_PARAMS MAX_SERVO

// Parameters and non-volatile state

#define CoL_degree 4
#define FF_degree 2
#define FuelFlow_degree 2

#define PARAM_VERSION 5U

struct ParamRecord {
  uint16_t crc;
  uint16_t version;
  char name[NAME_LEN+1];
  float dimension;
  AS5048_word_t alphaRef;
  float alphaOffset;
  float alphaMax[2];
  float maxPitch;
  float thresholdMargin, pushMargin, shakerMargin;
  float flare;
  int8_t functionMap[MAX_SERVO]; 
  float neutral[MAX_SERVO]; 
  float aileDefl;
  float elevDefl;
  float flapDefl;
  float rudderDefl;
  float steerDefl, steerPark, steerTrim;
  float brakeDefl;
  float canardDefl, canardGain, canardGainD;
  float vertDefl;
  float horizDefl;
  float i_Ku_C, i_Tu, o_P;
  float s_Ku_C, s_Tu;
  float r_Ku_C, r_Tu;
  float r_Mix;
  float yd_C;
  float t_Mix, t_Expo;
  float roll_C, roll_Expo;
  float coeff_FF[2][FF_degree+1];
  float coeff_CoL[2][CoL_degree+1];
  float servoRate;
  float takeoffTrim;
  float weightDry, fuel, battery, thrust;
  float fuelDensity;
  bool virtualOnly;
  bool haveGear;
  bool wowCalibrated;
  bool sensorOrient;
  float coeff_Flow[FuelFlow_degree+1];
  float idle, lag;
  uint16_t doorDelay, gearDelay, gearSpeed;
};

struct DerivedParams {
  bool valid;
  float takeoffMass;
  bool haveRetracts, haveFlaps;
  float assumedFlap, assumedMass;
  float coeff_FF[FF_degree+1];
  float coeff_CoL[CoL_degree+1];
  float minimumIAS, minimumDynP, zeroLiftAlpha,
    maxCoeffOfLift, maxCoeffOfLiftClean, maxCoeffOfLiftLand;
  float thresholdAlpha, shakerAlpha, pusherAlpha, maxAlpha, stallAlpha;
  float apexElev, apexAlpha;
};

#define MAX_TESTS  4

struct NVStateRecord {
  uint16_t crc;
  uint16_t paramPartition, dataPartition, logPartition;
  uint16_t logStamp;
  uint16_t model;
  int16_t testNum[MAX_TESTS];
  uint16_t rxCenter[MAX_CH], rxMin[MAX_CH], rxMax[MAX_CH];
};

extern struct ParamRecord vpParam;
extern struct NVStateRecord nvState;
extern struct DerivedParams vpDerived;

void defaultParams(void);
bool setModel(int model, bool verbose);
void deleteModel(int model);
void storeParams(void);
bool readNVState(void);
void storeNVState(void);
void printParams(void);
void derivedValidate(void);
void derivedInvalidate(void);
int maxModels(void);
void readData(uint8_t *data, int size);
void storeData(const uint8_t *data, int size);
  
#endif
