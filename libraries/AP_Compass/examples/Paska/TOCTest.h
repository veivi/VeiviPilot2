extern "C" {
#include "Math.h"
}

//
// Takeoff configuration test
//

typedef enum {
  toc_alpha,
  toc_pitot,
  toc_link,
  toc_lstick,
  toc_rstick,
  toc_tuning,
  toc_button,
  toc_attitude,
  toc_gyro,
  toc_mode,
  toc_fdr,
  toc_ram,
  toc_load,
} testCode_t;

#define TOC_TEST_NAME_MAX 16

struct TakeoffTest {
  char description[TOC_TEST_NAME_MAX];
  bool (*function)(bool);
};

const float toc_margin_c = RATIO(3/100);

void tocTestUpdate();
void tocReportConsole(bool result, int i, const char *s);
bool tocTestStatus(void (*reportFn)(bool, int, const char*));
bool tocTestReset();
