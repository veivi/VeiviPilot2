#!/bin/bash

target=$1

if [ -z "$target" ]
then
    echo "Target specifier missing."
    exit 0
fi

if [ ! -x $ALPHAPILOT/AlphaPilot/TARGET/INAV/$target ]
then
    echo "Target INAV/$target not found."
    exit 0
fi

if [ ! -x src/main/AlphaPilot/TARGET/$target ]
then
    mkdir -p src/main/AlphaPilot/TARGET/$target
fi

update="$ALPHAPILOT/AlphaPilot/update_file.bash"

$update "$ALPHAPILOT/AlphaPilot/*.h" src/main/AlphaPilot
$update "$ALPHAPILOT/AlphaPilot/*.c" src/main/AlphaPilot
$update $ALPHAPILOT/AlphaPilot/TARGET/INAV/$target/StaP_TARGET.h src/main/AlphaPilot/TARGET/$target
$update $ALPHAPILOT/AlphaPilot/TARGET/INAV/$target/StaP.c src/main/AlphaPilot/TARGET/$target
$update $ALPHAPILOT/AlphaPilot/TARGET/INAV/main.c src/main
$update $ALPHAPILOT/AlphaPilot/TARGET/INAV/CMakeLists.txt src/main
$update $ALPHAPILOT/AlphaPilot/TARGET/INAV/main.cmake cmake

# $update $ALPHAPILOT/AlphaPilot/TARGET/INAV/bus_i2c.h src/main/drivers
# $update $ALPHAPILOT/AlphaPilot/TARGET/INAV/bus_i2c_impl.h src/main/drivers
# $update $ALPHAPILOT/AlphaPilot/TARGET/INAV/$target/bus_i2c_stm32f10x.c src/main/drivers



