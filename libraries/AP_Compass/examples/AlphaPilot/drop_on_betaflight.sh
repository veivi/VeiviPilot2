#!/bin/bash

target=$1

if [ -z "$target" ]
then
    echo "Target specifier missing."
    exit 0
fi

if [ ! -x $ALPHAPILOT/AlphaPilot/TARGET/BF/$target ]
then
    echo "Target BF/$target not found."
    exit 0
fi


if [ ! -x src/main/AlphaPilot ]
then
    mkdir -p src/main/AlphaPilot/TARGET/BF
fi

update="$ALPHAPILOT/AlphaPilot/update_file.bash"

$update "$ALPHAPILOT/AlphaPilot/*.h" src/main/AlphaPilot
$update "$ALPHAPILOT/AlphaPilot/*.c" src/main/AlphaPilot
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/$target/StaP_TARGET.h src/main/AlphaPilot/TARGET/BF
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/$target/StaP.c src/main/AlphaPilot/TARGET/BF
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/main.c src/main
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/bus_i2c.h src/main/drivers
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/bus_i2c_impl.h src/main/drivers
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/$target/bus_i2c_stm32f10x.c src/main/drivers
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/source.mk make
