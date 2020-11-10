#!/bin/bash

if [ ! -x src/main/AlphaPilot ]
then
    mkdir -p src/main/AlphaPilot/TARGET/INAV
fi

update="$ALPHAPILOT/AlphaPilot/update_file.bash"

$update "$ALPHAPILOT/AlphaPilot/*.h" src/main/AlphaPilot
$update "$ALPHAPILOT/AlphaPilot/*.c" src/main/AlphaPilot
$update "$ALPHAPILOT/AlphaPilot/TARGET/INAV/*.h" src/main/AlphaPilot/TARGET/INAV
$update "$ALPHAPILOT/AlphaPilot/TARGET/INAV/*.c" src/main/AlphaPilot/TARGET/INAV
$update $ALPHAPILOT/AlphaPilot/TARGET/INAV/main.c src/main
$update $ALPHAPILOT/AlphaPilot/TARGET/INAV/CMakeLists.txt src/main
$update $ALPHAPILOT/AlphaPilot/TARGET/INAV/main.cmake cmake

