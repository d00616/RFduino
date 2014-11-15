#!/bin/sh
#
# JLink Flash Script for rfduino with plain nRF51 chips
# Copyright (C) 2014 d00616
# 
# This Software is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This Software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

set >/tmp/set.txt

# Params for JLinkExe
JLINKPARAMS="-Device nrf51822 -speed 100 -if swd"

# Mapfile
MAPFILE="${1}.map"

# Elf File
ELFFILE="${1}.elf"

# Binfile
BINFILE="${1}.bin"

# ARMGCCPATH
for ARMGCCPATH in $(pwd)/hardware/tools/gcc-arm-none-eabi-4.8.3-2014q1/bin /opt/arduino/hardware/tools/gcc-arm-none-eabi-4.8.3-2014q1/bin $(dirname $(which arm-none-eabi-objcopy)); do
	test -e "ARMGCCPATH/arm-none-eabi-objcopy" && break
done

# Find JLinkExe
jlinkexevariants="$(which JLinkExe) /opt/SEGGER/JLink/JLinkExe"
for JLINKEXE in ${jlinkexevariants}; do
	test -e "${JLINKEXE}" && break
done

if [ ! -e "${JLINKEXE}" ]; then
	echo "Please Install JLinkExe: http://www.segger.com/" >&2
	exit 99
fi

# Check files
if [ ! -e "${MAPFILE}" ]; then
	echo "Need ${MAPFILE}" >&2
	exit 1
fi
if [ ! -e "${ELFFILE}" ]; then
	echo "Need ${ELFFILE}" >&2
	exit 1
fi
if [ ! -e "${ARMGCCPATH}/arm-none-eabi-objcopy" ]; then
	echo "Need arm-none-eabi-objcopy" >&2
	exit 1
fi

# Create binary
"${ARMGCCPATH}/arm-none-eabi-objcopy" -O binary "${ELFFILE}" "${BINFILE}"
if [ ! -e "${BINFILE}" ]; then
	echo "Cannot generate bin file" >&2
	exit 2
fi

# Calculate Size
filesize=$(stat -c %s "${BINFILE}")
flashstart=$(cat "${MAPFILE}" | grep ^FLASH | awk '{ print $2}')

# write flash script
script=$(mktemp)

# Erase pages
addr=$(printf "%d" "${flashstart}")
while [ "$filesize" -gt 0 ]; do
	# erase memory page
	echo "w4 0x4001e508, "$(printf "0x%x" "${addr}") >> "${script}"
        filesize=$(expr ${filesize} - 1024)
        addr=$(expr ${addr} + 1024)
done

cat >>"${script}" <<EOF
sleep 1000
w4 0x4001e504,1
loadbin "${BINFILE}", $flashstart
verifybin "${BINFILE}", $flashstart
r
g
qc
EOF

# debug
#cat "$script"

"${JLINKEXE}" ${JLINKPARAMS} -CommanderScript "${script}"
ret=$?
test -f "${script}" && rm "${script}"
exit "${ret}"
