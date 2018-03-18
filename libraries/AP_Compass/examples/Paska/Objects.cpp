#include "Objects.h"

struct ModeRecord vpMode;
struct FeatureRecord vpFeature;
struct StatusRecord vpStatus;
struct FlightState vpFlight;
struct InputState vpInput;
struct ControlState vpControl;
struct OutputState vpOutput;
// struct GPSFix gpsFix;

float controlCycle;
float outer_P, rudderMix, throttleMix;
uint8_t flapSel, gearSel;
