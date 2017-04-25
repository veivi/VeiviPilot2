#include "Status.h"

//
//
//

bool pitotFailed()
{
  return !vpStatus.simulatorLink && pitotDevice.status();
}

bool alphaFailed()
{
  return vpStatus.alphaLocked
    || (!vpStatus.simulatorLink && alphaDevice.status());
}

