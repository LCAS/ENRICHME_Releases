#!/bin/bash

read -p "Make sure AirBox is unplugged and then press enter" dummy
unplugged="$(ls /dev | grep "ttyUSB")"
read -p "Now plug AirBox in and then press enter" dummy
plugged="$(ls /dev | grep "ttyUSB")"

fileList="$(diff <(echo "$unplugged") <(echo "$plugged"))"

array=(${fileList// / })

for element in "${array[@]}"
do
	if [[ $element == "ttyUSB"* ]]
	then
		file="$element"
	fi
done


if [ -z "$file" ]
then
  echo "Airbox not detected as usb device !!!!"
  exit 1 # terminate and indicate error
fi

echo "AirBox detected at: $file"

# to let system detect it as /ttyUSB...
#vendor and product id
#sudo modprobe ftdi-sio
#sudo echo "$vend" "$prod" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id


prod="$(udevadm info -a -n /dev/$file | grep '{idProduct}' | head -n1 | tr -d '[[:space:]]')"
vend="$(udevadm info -a -n /dev/$file | grep '{idVendor}' | head -n1 | tr -d '[[:space:]]')"
seri="$(udevadm info -a -n /dev/$file | grep '{serial}' | head -n1 | tr -d '[[:space:]]')"

