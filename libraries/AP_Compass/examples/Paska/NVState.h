#ifndef NVSTATE_H
#define NVSTATE_H

#include <stdint.h>
#include <stdbool.h>

#define NAME_LEN     8
#define MAX_CH       8
#define MAX_SERVO    11
#define MAX_PARAMS MAX_SERVO

typedef enum {
  fn_null,
  fn_aileron,
  fn_elevator,
  fn_rudder,
  fn_throttle,
  fn_gear,
  fn_steering,
  fn_brake,
  fn_leftaileron, fn_rightaileron,
  fn_leftcanard, fn_rightcanard,
  fn_leftelevon, fn_rightelevon,
  fn_lefttail, fn_righttail,
  fn_leftflap, fn_rightflap,
  fn_leftthrustvert, fn_rightthrustvert,
  fn_thrusthoriz,
  fn_gearinv,
  fn_invalid
} function_t;

// Parameters and non-volatile state

#define CoL_degree 4
#define FF_degree 2

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
  float steerNeutral, steerDefl, steerPark;
  float brakeNeutral, brakeDefl;
  float canardNeutral, canardDefl;
  float vertNeutral, vertDefl;
  float horizNeutral, horizDefl;
  uint8_t functionMap[MAX_SERVO]; 
  float alphaMax[2];
  float i_Ku_C, i_Tu, o_P;
  float s_Ku_C, s_Tu;
  float r_Mix;
  float at_Ku, at_Tu;
  float cc_Ku, cc_Tu;
  float coeff_FF[2][FF_degree+1];
  float t_Mix, t_Expo;
  float maxPitch;
  float roll_C;
  float coeff_CoL[2][CoL_degree+1];
  float servoRate;
  float takeoffTrim;
  float weightDry, fuel, thrust;
  float thresholdMargin, pushMargin;
  float yd_C;
  float offset;
  bool flaperon;
  bool virtualOnly;
  bool haveGear;
  bool wowCalibrated;
  bool sensorOrient;
  float expo;
  int16_t floor;
  float flare;
  bool gearLock;
  };

struct DerivedParams {
  bool haveRetracts, haveFlaps;
  float totalMass;
  float assumedFlap;
  float coeff_FF[FF_degree+1];
  float coeff_CoL[CoL_degree+1];
  float minimumIAS, minimumDynP, zeroLiftAlpha,
    maxCoeffOfLift, maxCoeffOfLiftClean;
  float thresholdAlpha, shakerAlpha, pusherAlpha, maxAlpha, stallAlpha;
  float apexElev, apexAlpha;
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

void defaultParams(void);
bool setModel(int model, bool verbose);
void deleteModel(int model);
void storeParams(void);
bool readNVState(void);
void storeNVState(void);
void printParams(void);
void deriveParams();
int maxModels(void);
void readData(uint8_t *data, int size);
void storeData(const uint8_t *data, int size);
  
#endif
