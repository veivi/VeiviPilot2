#ifndef NVSTATE_H
#define NVSTATE_H

#include <stdint.h>
#include <AP_ProgMem/AP_ProgMem.h>

#define NAME_LEN     8
#define MAX_CH       8
#define MAX_SERVO    11

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
  fn_invalid
} function_t;

// Parameters and non-volatile state

struct ParamRecord {
  uint16_t crc;
  char name[NAME_LEN+1];
  uint8_t i2c_clkDiv;
  uint8_t i2c_5048B, i2c_24L256;
  uint16_t alphaRef, airSpeedRef;
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
  float ff_A[2], ff_B[2], ff_C[2];
  float t_Mix, t_Expo;
  float maxPitch;
  float roll_C;
  float cL_A[2], cL_B[2], cL_C[2], cL_D[2], cL_E[2];
  float servoRate;
  float takeoffTrim;
  float weightDry, fuel, thrust;
  float thresholdMargin, stallMargin;
  float glideSlope;
  float offset;
  bool flaperon;
  bool virtualOnly;
  bool haveGear;
  bool wowCalibrated;
  bool sensorOrient;
  float expo;
  int16_t floor;
  };

struct DerivedParams {
  bool haveRetracts, haveFlaps;
  float totalMass;
  float assumedFlap;
  float ff_A, ff_B, ff_C;
  float cL_A, cL_B, cL_C, cL_D, cL_E;
  float minimumIAS, minimumDynP, zeroLiftAlpha, maxCoeffOfLift;
  float thresholdAlpha, shakerAlpha, pusherAlpha, maxAlpha;
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
extern const prog_char_t *updateDescription;

void defaultParams(void);
bool setModel(int model, bool verbose);
void deleteModel(int model);
void storeParams(void);
bool readNVState(void);
void storeNVState(void);
void printParams(void);
void deriveParams();
void backupParams(void);
int maxModels(void);
void readData(uint8_t *data, int size);
void storeData(const uint8_t *data, int size);
  
#endif
