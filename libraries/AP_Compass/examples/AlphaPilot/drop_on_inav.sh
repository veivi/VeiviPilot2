#!/bin/bash

if [ ! -x src/main/AlphaPilot ]
then
    mkdir -p src/main/AlphaPilot/TARGET/INAV
fi

cp -f $ALPHAPILOT/AlphaPilot/*.h src/main/AlphaPilot/
cp -f $ALPHAPILOT/AlphaPilot/*.c src/main/AlphaPilot/
cp -f $ALPHAPILOT/AlphaPilot/TARGET/INAV/*.h src/main/AlphaPilot/TARGET/INAV/
cp -f $ALPHAPILOT/AlphaPilot/TARGET/INAV/*.c src/main/AlphaPilot/TARGET/INAV/
cp -f $ALPHAPILOT/AlphaPilot/TARGET/INAV/main.c src/main/
cp -f $ALPHAPILOT/AlphaPilot/TARGET/INAV/CMakeLists.txt src/main/
cp -f $ALPHAPILOT/AlphaPilot/TARGET/INAV/main.cmake cmake/

