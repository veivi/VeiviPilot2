#!/bin/bash

for file in $1
do
    base=$(basename $file)
    
    if [ "$file" -nt "$2/$base" ]
    then
	echo "Updating from $file to $2/$base"
	cp "$file" "$2/$base"
    fi
done

