#!/bin/bash

if [ ! -x src/main/AlphaPilot ]
then
    mkdir -p src/main/AlphaPilot/TARGET/BF
fi

update="$ALPHAPILOT/AlphaPilot/update_file.bash"

$update "$ALPHAPILOT/AlphaPilot/*.h" src/main/AlphaPilot
$update "$ALPHAPILOT/AlphaPilot/*.c" src/main/AlphaPilot
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/StaP_TARGET.h src/main/AlphaPilot/TARGET/BF
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/StaP.c src/main/AlphaPilot/TARGET/BF
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/main.c src/main
$update $ALPHAPILOT/AlphaPilot/TARGET/BF/source.mk make
