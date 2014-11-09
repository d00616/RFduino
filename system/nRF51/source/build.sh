#!/bin/sh

# Go to source directory
cd $(dirname $0)

# define variables
arduino=$(pwd | sed 's/\/hardware\/arduino\/RFduino\/system\/nRF51\/source$//')
tools="$arduino/hardware/tools/gcc-arm-none-eabi-4.8.3-2014q1"
gcc="$tools/bin/arm-none-eabi-gcc"
as="$tools/bin/arm-none-eabi-as"
ar="$tools/bin/arm-none-eabi-ar"
nm="$tools/bin/arm-none-eabi-nm"
includes="-I../include -I../../nRF51-SDK/nrf51822/Include/ -I../../CMSIS/CMSIS/Include"
cflags="-Os -w -mcpu=cortex-m0 -mthumb -DNRF51 -DBOARD_PCA10001 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-builtin $includes"
startupcode="../../nRF51-SDK/nrf51822/Source/templates/gcc/gcc_startup_nrf51.s"

# create build directory
build=$(mktemp -d) || exit 1

# compile
echo "compiling..."

#echo "startup_nrf51822.c"
#$gcc $cflags -c -o "$build/startup_nrf51822.o" "startup_nrf51822.c" || exit 2
echo "startup code"
$as -o "$build/startup_nrf51822.o" "$startupcode" || exit 2

echo "system_nrf51.c"
$gcc $cflags -c -o "$build/system_nrf51.o" "../../nRF51-SDK/nrf51822/Source/templates/system_nrf51.c" || exit 2

echo "creating library..."
objs="$build/startup_nrf51822.o $build/system_nrf51.o"
output="libnRF51System.a"

for f in $objs; do
	$ar rcs "$build/$output" "$f" || exit 3
done

$nm "$build/$output" >"$build/""$output"".txt" || exit 4

echo "copying libray and txt to variants..."
cp "$build/$output" "../../../variants/nRF51/$output" || exit 5
cp "$build/$output"".txt" "../../../variants/nRF51/$output"".txt" || exit 5

# remove build directory
test -d "$build" && rm -rf "$build"
