#!/bin/sh
#
# Copyright (c) 2014 nRF51duino.  All right reserved.
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
tools=/opt/gcc-arm-none-eabi-4_8-2014q1/
gpp="$tools/bin/arm-none-eabi-g++"
gcc="$tools/bin/arm-none-eabi-gcc"
as="$tools/bin/arm-none-eabi-as"
ar="$tools/bin/arm-none-eabi-ar"
nm="$tools/bin/arm-none-eabi-nm"
ld="$tools/bin/arm-none-eabi-ld"
includes=""
#cflags="-Os -w -mcpu=cortex-m0 -mthumb -mabi=aapcs -DNRF51 -DBOARD_PCA10001 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-builtin $includes"
#cflags="-Os -w -mcpu=cortex-m0 -mthumb -mabi=aapcs -mfloat-abi=soft -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-builtin -DNRF51 -DBOARD_PCA10001 $includes"
cflags="-c -g -Os -w -mcpu=cortex-m0 -mthumb -mabi=aapcs -mfloat-abi=soft -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-builtin -mcpu=cortex-m0 -DF_CPU=16000000"
ldflags="-Wl,--gc-sections --specs=nano.specs -mcpu=cortex-m0 -mthumb -march=armv6-m"
#startupcode="../../nRF51-SDK/nrf51822/Source/templates/gcc/gcc_startup_nrf51.s"
#startupcode="gcc_startup_nrf51.s"

# create build directory
#build=$(mktemp -d) || exit 1
build="_build"
test -e "$build" || mkdir "$build"

# compile
echo "compiling..."

ldfiles=""
for file in g++_startup main; do
	echo "${file}.c" -Wl,--end-group
	$gpp $cflags -o "$build/${file}.o" "${file}.c" || exit 2
	ldfiles="${ldfiles} $build/${file}.o"
done

$gpp $ldflags -TARMCMx.ld -Wl,-Map,${build}/out.map -Wl,--cref -o ${build}/out.elf -L${build} -Wl,--warn-common -Wl,--warn-section-align -Wl,--start-group ${ldfiles} -Wl,--end-group

# remove build directory
#test -d "$build" && rm -rf "$build"
