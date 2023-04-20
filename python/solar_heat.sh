#!/bin/bash

# ttyAMA0 on pin GPIO14 GPIO15
raspi-gpio set 14 a0
raspi-gpio set 15 a0
raspi-gpio set 32 ip
raspi-gpio set 33 ip

sleep 1

while [ true ]
do
	socat /dev/ttyAMA0,raw,echo=0,nonblock,min=0,b115200 STDIO | python3 solar_heat_home_assistant.py
	sleep 1
done
