#include <string.h>
#include "TOCTest.h"
#include "Button.h"

extern "C" {
#include "CoreObjects.h"
#include "Logging.h"
#include "RxInput.h"
#include "Storage.h"
#include "Console.h"
#include "Time.h"
#include "NVState.h"
#include "DSP.h"
#include "Math.h"
}

bool toc_test_mode(bool reset)
{
  return !vpMode.test
    && vpMode.wingLeveler && vpMode.bankLimiter && vpMode.takeOff;
}

bool toc_test_link(bool reset)
{
  return currentTime - lastPPMWarn > 10e6 &&
    (vpParam.virtualOnly || ppmFreq > 30) &&
    (!vpParam.virtualOnly || vpStatus.simulatorLink);
}

bool toc_test_ram(bool reset)
{
  return memoryFree() > (1<<9);
}

bool toc_test_load(bool reset)
{
  return idleAvg > 0.15;
}

bool toc_test_fdr(bool reset)
{
  return eepromIsOnline() && logReady(false);
}

bool toc_test_alpha_sensor(bool reset)
{
  return !vpStatus.alphaFailed && damperOutput(&alphaEntropy) > 50
    && fieldStrength > 0.15 && fieldStrength < 0.80;
}

bool toc_test_alpha_range(bool reset)
{
  static bool bigAlpha, zeroAlpha;
  static uint32_t lastNonZeroAlpha, lastSmallAlpha;

  if(reset) {
    zeroAlpha = bigAlpha = false;
    lastNonZeroAlpha = lastSmallAlpha = currentTime;
    
  } else if(!zeroAlpha) {
    if(fabs(vpFlight.alpha) > 1.5/RADIAN) {
      lastNonZeroAlpha = currentTime;
    } else if(currentTime > lastNonZeroAlpha + 1.0e6) {
      consoleNoteLn_P(CS_STRING("Stable ZERO ALPHA"));
      zeroAlpha = true;
    }
  } else if(!bigAlpha) {
    if(fabsf(vpFlight.alpha - 90/RADIAN) > 30/RADIAN) {
      lastSmallAlpha = currentTime;
    } else if(currentTime > lastSmallAlpha + 1.0e6) {
      consoleNoteLn_P(CS_STRING("Stable BIG ALPHA"));
      bigAlpha = true;
    }
  }
  
  return (bigAlpha && zeroAlpha);
}

bool toc_test_alpha(bool reset)
{
  return (toc_test_alpha_sensor(reset) && toc_test_alpha_range(reset))
     || vpStatus.simulatorLink;
}

bool toc_test_pitot(bool reset)
{
  static bool positiveIAS;
  
  if(reset)
    positiveIAS = false;
  else if(vpStatus.positiveIAS)
    positiveIAS = true;
  
  return (!vpStatus.pitotFailed && damperOutput(&iasEntropy) > 50
	  && !vpStatus.pitotBlocked && positiveIAS && vpFlight.IAS < 5)
    || vpStatus.simulatorLink;
}

bool toc_test_attitude(bool reset)
{
  return fabsf(vpFlight.pitch) < 10/RADIAN && fabsf(vpFlight.bank) < 5/RADIAN;
}

bool toc_test_gyro(bool reset)
{
  return vpStatus.simulatorLink
    || (fabsf(vpFlight.pitchR) < 1.0/RADIAN
	&& fabsf(vpFlight.rollR) < 1.0/RADIAN
	&& fabsf(vpFlight.yawR) < 1.0/RADIAN);
}

struct TOCRangeTestState {
  float valueMin, valueMax;
};

bool toc_test_range_generic(struct TOCRangeTestState *state, bool reset, struct RxInputRecord *input, float expectedMin, float expectedMax)
{
  const float value = inputValue(input);
  
  if(reset) {
    state->valueMin = state->valueMax = value;
    return true;
  } else {
    state->valueMin = fminf(state->valueMin, value);
    state->valueMax = fmaxf(state->valueMax, value);
  }
  
  return (fabsf(state->valueMin - expectedMin) < toc_margin_c
	  && fabsf(state->valueMax - expectedMax) < toc_margin_c);
}

bool toc_test_rstick_range(bool reset)
{
  static struct TOCRangeTestState stateElev, stateAile;
  return toc_test_range_generic(&stateElev, reset, &elevInput, -1, 1)
    && toc_test_range_generic(&stateAile, reset, &aileInput, -1, 1);
}

bool toc_test_rstick_neutral(bool reset)
{
  return ( fabsf(inputValue(&aileInput)) < toc_margin_c )
    && ( fabsf(inputValue(&elevInput)) < toc_margin_c );
}

bool toc_test_rstick(bool reset)
{
  return toc_test_rstick_range(reset) && toc_test_rstick_neutral(reset);
}

bool toc_test_lstick_range(bool reset)
{
  static struct TOCRangeTestState stateThr;
  bool status = toc_test_range_generic(&stateThr, reset, &throttleInput, 0, 1);

#if RX_CHANNELS >= 8
  static struct TOCRangeTestState stateRudder;
  bool status2 = toc_test_range_generic(&stateRudder, reset, &rudderInput, -1, 1);
  status = status && status2;
#endif

  return status;
}

bool toc_test_lstick_neutral(bool reset)
{
  bool status = fabsf(inputValue(&throttleInput)) < toc_margin_c;
    
#if RX_CHANNELS >= 8
  status = status && fabsf(inputValue(&rudderInput)) < toc_margin_c;
#endif
  
  return status;
}

bool toc_test_lstick(bool reset)
{
  return toc_test_lstick_range(reset) && toc_test_lstick_neutral(reset);
}

bool toc_test_tuning_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &tuningKnobInput, 0, 1);
}

bool toc_test_tuning_zero(bool reset)
{
  return fabsf(inputValue(&tuningKnobInput)) < toc_margin_c;
}

bool toc_test_tuning(bool reset)
{
  return toc_test_tuning_range(reset) && toc_test_tuning_zero(reset);
}

bool toc_test_button_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &buttonInput, -1, 1);
}

bool toc_test_button_neutral(bool reset)
{
  return  !leftUpButton.state() && !leftDownButton.state()
    && !rightUpButton.state() && !rightDownButton.state();
}

bool toc_test_button(bool reset)
{
  return toc_test_button_range(reset) && toc_test_button_neutral(reset);
}

const struct TakeoffTest tocTest[] CS_QUALIFIER =
  {
    [toc_alpha] = { "ALPHA", toc_test_alpha },
    [toc_pitot] = { "PITOT", toc_test_pitot },
    [toc_link] = { "LINK", toc_test_link },
    [toc_lstick] = { "LSTIK", toc_test_lstick },
    [toc_rstick] = { "RSTIK", toc_test_rstick },
    [toc_tuning] = { "TUNE", toc_test_tuning },
    [toc_button] = { "BUTTN", toc_test_button },
    [toc_attitude] = { "ATTI", toc_test_attitude },
    [toc_gyro] = { "GYRO", toc_test_gyro },
    [toc_mode] = { "MODE", toc_test_mode },
    [toc_fdr] = { "DATA", toc_test_fdr },
    [toc_ram] = { "RAM", toc_test_ram },
    [toc_load] = { "LOAD", toc_test_load }
  };

const int tocNumOfTests = sizeof(tocTest)/sizeof(struct TakeoffTest);

static bool tocStatusFailed;

bool tocTestInvoke(bool reset, bool challenge, void (*report)(bool, int, const char *))
{
  struct TakeoffTest cache;

  tocStatusFailed = false;
  
  for(int i = 0; i < tocNumOfTests; i++) {
    memcpy_P(&cache, &tocTest[i], sizeof(cache));

    bool result = (*cache.function)(reset);
    
    if(challenge) {
      (*report)(result, i, cache.description); 

      if(!result)
	tocStatusFailed = true;
    }
  }

  return !tocStatusFailed;
}

bool tocTestReset()
{
  return tocTestInvoke(true, false, NULL);
}

void tocTestUpdate()
{
  tocTestInvoke(false, false, NULL);
}

void tocReportConsole(bool result, int i, const char *s)
{  
  if(!result) {
    if(!tocStatusFailed)
      consoleNote_P(CS_STRING("T/O/C FAIL :"));

    consolePrint(" ");
    consolePrint(s);
  }
}

bool tocTestStatus(void (*reportFn)(bool, int, const char*))
{
  return tocTestInvoke(false, true, reportFn);
}

