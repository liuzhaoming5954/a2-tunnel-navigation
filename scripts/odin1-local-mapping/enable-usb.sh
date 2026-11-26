#!/usr/bin/env bash
set -euo pipefail
device_info=$(lsusb -d 2207:0019)
if [[ ! "$device_info" =~ ^Bus\ ([0-9]{3})\ Device\ ([0-9]{3}):\ ID\ 2207:0019[[:space:]] ]] ||
   [[ "$device_info" = *$'\n'* ]]; then
    printf '%s\n' 'Exactly one ODIN1 device must be connected.' >&2
    exit 1
fi
device="/dev/bus/usb/${BASH_REMATCH[1]}/${BASH_REMATCH[2]}"
test -c "$device"
if ! test -w "$device"; then
    if command -v setfacl >/dev/null; then
        sudo setfacl -m "u:$(id -un):rw" "$device"
    else
        id -nG | tr ' ' '\n' | grep -Fxq plugdev
        sudo chgrp -- plugdev "$device"
    fi
fi
test -r "$device" && test -w "$device"
printf '%s\n' 'ODIN1 local USB access ready'
