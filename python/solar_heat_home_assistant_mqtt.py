import binascii 
import struct
import traceback
import requests
import time
import json
import sys
import serial
import paho.mqtt.client as mqtt

ser = serial.Serial(port="/dev/ttyAMA0", baudrate=115200)

mqtt_client = None

def solar_pump_disable(state):
  if state:
    print("pumping disabled: ON")
    ser.write(b'\xDE')
    mqtt_client.publish(  'homeassistant/switch/solar_pump_disable/state', payload="ON", retain=True) 
  else:
    print("pumping disabled: OFF")
    ser.write(b'\xEA')
    mqtt_client.publish(  'homeassistant/switch/solar_pump_disable/state', payload="OFF", retain=True) 

def on_message_solar_pump_disable(client, userdata, msg):
  try:
    if msg.payload.decode('utf-8') == "ON":
      solar_pump_disable(1)
    else:
      solar_pump_disable(0)
  except:
    traceback.print_exc()

def mqtt_start():
  global mqtt_client

  def mqtt_setup():
    global mqtt_client
    # actuation
    mqtt_client.message_callback_add('homeassistant/switch/solar_pump_disable/set', on_message_solar_pump_disable)
    # home assistant mqtt based auto discovery
    mqtt_client.publish(  'homeassistant/switch/solar_pump_disable/config', payload=json.dumps({"name": "Disable Solar Pumping", "command_topic": "homeassistant/switch/solar_pump_disable/set", "state_topic": "homeassistant/switch/solar_pump_disable/state" }), retain=True)
    mqtt_client.subscribe('homeassistant/switch/solar_pump_disable/set')
    # off by default (forced no)
    solar_pump_disable(0)

    # monitoring
    mqtt_client.publish(  'homeassistant/binary_sensor/solar_pumping/config', payload=json.dumps({"name": "Solar pump running", "state_topic": "homeassistant/binary_sensor/solar_pumping/state" }), retain=True)
    mqtt_client.publish(  'homeassistant/sensor/solar_temp_top/config', payload=json.dumps({"device_class": "temperature", "name": "Top Tank Temperature ", "state_topic": "homeassistant/sensor/solar_temp_top/state", "unit_of_measurement": "°C"}), retain=True)
    # no default value at boot mqtt_client.publish(  'homeassistant/sensor/solar_temp_top/state', payload="0", retain=True) #off by default (forced no)
    mqtt_client.publish(  'homeassistant/sensor/solar_temp_bottom/config', payload=json.dumps({"device_class": "temperature", "name": "Bottom Tank Temperature", "state_topic": "homeassistant/sensor/solar_temp_bottom/state", "unit_of_measurement": "°C"}), retain=True)
    mqtt_client.publish(  'homeassistant/sensor/solar_temp_panel/config', payload=json.dumps({"device_class": "temperature", "name": "Panel Temperature", "state_topic": "homeassistant/sensor/solar_temp_panel/state", "unit_of_measurement": "°C"}), retain=True)

  def on_connect(client, userdata, flags, rc):
    #print(f"on_connect: rc={rc}")
    if rc==0:
      mqtt_setup()

  mqtt_client = mqtt.Client('solar_heat_controller',clean_session=True)
  mqtt_client.on_connect=on_connect
  while True:
    try:
      mqtt_client.connect('192.168.0.4', 1883)
      mqtt_client.loop_start()
      break
    except:
      traceback.print_exc()
    time.sleep(1)

mqtt_start()

# read values from the solax
while True:
  try:
    data=b''
    while True:
      data = ser.read(1)
      #magic 1
      if data[0] == 0xAA:
        break
    data += ser.read(10)
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

    pumping='OFF'
    if fields[IDX_PUMP_STATE] > 0:
      pumping='ON'
    mqtt_client.publish('homeassistant/binary_sensor/solar_pumping/state', payload=pumping, retain=True)
    mqtt_client.publish('homeassistant/sensor/solar_temp_panel/state', payload=str(fields[IDX_TEMP_PANEL]/10.0), retain=True)
    mqtt_client.publish('homeassistant/sensor/solar_temp_top/state', payload=str(fields[IDX_TEMP_TOP]/10.0), retain=True)
    mqtt_client.publish('homeassistant/sensor/solar_temp_bottom/state', payload=str(fields[IDX_TEMP_BOTTOM]/10.0), retain=True)
  except:
    print(traceback.print_exc())
    sys.exit(-1)
  #time.sleep(0.1)#sys stdin is done wit ha timeout

#screen -dmS diemasol bash -c 'cd /automeshion/ ; bash solar_heat.sh' &
