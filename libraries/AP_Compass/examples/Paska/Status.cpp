#include "Status.h"

//
//
//

bool pitotFailed()
{
  return vpStatus.fault == 1
    || (!vpStatus.simulatorLink && pitotDevice.status());
}

bool alphaFailed()
{
  return vpStatus.alphaLocked
    || vpStatus.fault == 2
    || (!vpStatus.simulatorLink && alphaDevice.status());
}

