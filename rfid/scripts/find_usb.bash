#!/bin/bash
# @brief Script that finds USB port, USB UART converter is connected to
for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && exit
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && exit
        if [[ "$ID_SERIAL" == *"UART"* ]]; then 
			echo "/dev/$devname"
		fi
	)
done