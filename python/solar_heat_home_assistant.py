import binascii 
import struct
import traceback
import requests
import time
import json
import sys

#TODO grab parameter to reach home assistant from the command line parameters (and the sensor stem)
hacs_token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJjN2VlZGQ3MDg0Njk0YTA4OTE1MzU2NDI2MmM3YjQ3OCIsImlhdCI6MTY3MzU1NDg0MCwiZXhwIjoxOTg4OTE0ODQwfQ.HGhqPP9NKxneoEEuhtwGtFBRXEjrF0mKNqHPQ5ekczE"
hacs_url = "http://192.168.0.4:8123"

# read stdin with data from the diemasol usart output
# socat /dev/ttyAMA0,raw,echo=0,nonblock,min=0,b115200 STDIO | stdbuf -i0 -o0 xxd -p | python3 solar_heat_home_assistant.py

def home_assistant_push(ident, name, value, unit=None):
  if not unit:
    requests.post(
        hacs_url + "/api/states/" + ident,
        headers={
            "Authorization": "Bearer " + hacs_token,
            "content-type": "application/json",
        },
        data=json.dumps({"state": ""+str(value), "attributes": {"state_class": "total", "friendly_name": name, "unique_id": ident, "entity_id": ident}}),
    )
  else:
    requests.post(
      hacs_url + "/api/states/" + ident,
      headers={
          "Authorization": "Bearer " + hacs_token,
          "content-type": "application/json",
      },
      data=json.dumps({"state": ""+str(value), "attributes": {"state_class": "total", "unit_of_measurement": unit, "friendly_name": name, "unique_id": ident, "entity_id": ident}}),
    )


# read values from the solax
while True: 
  try:
    data=b''
    while True:
      data = sys.stdin.buffer.read(1)
      if data[0] == 0xAA:
        break
    data += sys.stdin.buffer.read(10)
    #             magic 2            total len
    if (data[1] != 0x55 or data[2] != 0x0B):
      continue

    print (binascii.hexlify(data))
    fields = struct.unpack_from(">BBBBhhhB", data)
    IDX_TEMP_PANEL = 4
    IDX_TEMP_TOP = 5
    IDX_TEMP_BOTTOM = 6
    IDX_PUMP_STATE = 7
    
    print (repr(fields))

    home_assistant_push("sensor.diemasol_pump",        "Solar pump running",      fields[IDX_PUMP_STATE]/10.0)
    home_assistant_push("sensor.diemasol_temp_panel",  "Panel temperature",       fields[IDX_TEMP_PANEL]/10.0, "°C")
    home_assistant_push("sensor.diemasol_temp_top",    "Tank top temperature",    fields[IDX_TEMP_TOP]/10.0, "°C")
    home_assistant_push("sensor.diemasol_temp_bottom", "Tank bottom temperature", fields[IDX_TEMP_BOTTOM]/10.0, "°C")

  except:
    print(traceback.print_exc())
    sys.exit(-1)
  #time.sleep(0.1)#sys stdin is done wit ha timeout

'''
/etc/rc.local
screen -dmS diemasol bash -c 'cd /automeshion/ ; bash solar_heat.sh' &

/automeshion/solar_heat.sh
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
'''