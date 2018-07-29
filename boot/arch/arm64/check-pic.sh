#!/bin/bash

# Poor man's checker that no relocatable data are present in given object files.
# Usage: check-pic.sh <objdump> [objfile...]
#
# This script is used to check all object files that compose image.raw. Its
# purpose is to verify that no dynamic relocations are present in data sections
# of these input files which means they do not need to be relocated at runtime.
# Since these sections typically also do not require any static relocations, the
# implemented test simply checks that there are no relocation sections
# associated with any data sections.

objdump=$1
shift

while [ "$#" -ge 1 ] ; do
	"$objdump" --section-headers "$1" | \
	    grep -E '\.rel\.data|\.rel\.rodata|\.rela\.data|\.rela\.rodata|\.data\.rel' \
	    > /dev/null 2>&1
	if [ $? -eq 0 ]; then
		echo "Error: Object $1 contains relocatable data" >&2
		exit 1
	fi
	shift
done
