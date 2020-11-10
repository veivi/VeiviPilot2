/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "fc/init.h"
#include "common/time.h"
#include "flight/servos.h"

#ifndef ALPHAPILOT

#include "scheduler/scheduler.h"

void run(void);

int main(void)
{
    init();

    run();

    return 0;
}

void FAST_CODE FAST_CODE_NOINLINE run(void)
{
    while (true) {
      scheduler();
      processLoopback();
#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif
    }
}

#else

#include <AlphaPilot/MainLoop.h>
#include <AlphaPilot/Datagram.h>
#include <AlphaPilot/Console.h>
#include <AlphaPilot/MS4525.h>
#include <AlphaPilot/StaP.h>

int main(void)
{
    init();

    mainLoopSetup();
    mainLoop();
    
    return 0;
}

#endif
