#!/bin/sh
#
# Copyright (c) 2014 D00616..  All right reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Go to source directory
cd $(dirname $0)

# define variables
arduino=$(pwd | sed 's/\/hardware\/arduino\/nRF51duino\/system\/nRF51\/source$//')
tools="$arduino/hardware/tools/gcc-arm-none-eabi-4.8.3-2014q1"
gcc="$tools/bin/arm-none-eabi-gcc"
as="$tools/bin/arm-none-eabi-as"
ar="$tools/bin/arm-none-eabi-ar"
nm="$tools/bin/arm-none-eabi-nm"
includes="-I../include -I../../nRF51-SDK/nrf51822/Include/ -I../../CMSIS/CMSIS/Include -I../../../variants/nRF51/"
cflags="-Os -w -mcpu=cortex-m0 -mthumb -DNRF51 -DBOARD_PCA10001 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-builtin $includes"
startupcode="../../nRF51-SDK/nrf51822/Source/templates/gcc/gcc_startup_nrf51.s"

# create build directory
build=$(mktemp -d) || exit 1

# compile
echo "compiling..."

echo "startup_nrf51822.c"
$gcc $cflags -c -o "$build/startup_nrf51822_c.o" "startup_nrf51822.c" || exit 2
echo "startup code"
$as -o "$build/startup_nrf51822_s.o" "$startupcode" || exit 2

echo "system_nrf51.c"
$gcc $cflags -c -o "$build/system_nrf51.o" "../../nRF51-SDK/nrf51822/Source/templates/system_nrf51.c" || exit 2

echo "libnRF51duino.c"
$gcc $cflags -c -o "$build/libnRF51duino.o" "libnRF51duino.c" || exit 2

echo "creating library..."
objs="$build/startup_nrf51822_s.o $build/startup_nrf51822_c.o $build/system_nrf51.o $build/libnRF51duino.o"
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
