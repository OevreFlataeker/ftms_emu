Used SDK 17.0.2
DevKit: NRF52DK

D:\nRF5_SDK_17.0.2_d674dde\examples\ble_central_and_peripheral\experimental\ble_app_cps_relay

Most of the headers comments have been explicitly removed in order to avoid confusion which parts are
genuine Nordic and which not

All 3rd party contrib stuff has been removed (FIT files, PCAP, BLE Sniffer, ...)

All Spec PDFs have been removed


---
---
---



Merged back to master

Do not forget to add the services 0x1814 (RCS) or 0x1816 (CSC) to the ADV Data! 
Otherwise it won't be found!

Macros
#define 	PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING   (PM_CONN_SEC_ERROR_BASE + 0x06)
 	Encryption failed because the peripheral has lost the LTK for this bond. See also BLE_HCI_STATUS_CODE_PIN_OR_KEY_MISSING and Table 3.7 ("Pairing Failed Reason Codes") in the Bluetooth Core Specification 4.2, section 3.H.3.5.5 (Bluetooth Core Specification).
	
#define 	PM_CONN_SEC_ERROR_DISCONNECT   (PM_CONN_SEC_ERROR_BASE + 0x100)
 	Pairing or encryption did not finish before the link disconnected for an unrelated reason.

When bonding fails, press Button 2 + RESET to delete bonding data.

Specifications:
https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.csc_measurement.xml
https://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=261450

At the moment this whole project needs to be placed into

$SDKROOT\examples\ble_central_and_peripheral\experimental\ble_app_cps_relay
because of the relative include directory setup

e.g. D:\nRF5_SDK_17.0.2_d674dde\examples\ble_central_and_peripheral\experimental\ble_app_cps_relay

Bike physics: https://www.physik.uni-wuerzburg.de/fileadmin/11010700/_imported/fileadmin/11010700/Didaktik/Zulassungsarbeiten/HA_1622196_Bielmeier_Carsten.pdf?
Section 5.2.3 - Free rolling

Formular free roll: v(t)=45*exp(-0.04*t)

Power measurement series:


Level 1:
76,58
77,59
79,61
80,62

Level 2:
74,67
75,69
77,71
78,72
79,73
80,75

Level 3:
75,80
76,81
79,84
80,86

Level 4:
76,94
77,96
78,98
79,100
80,102

Level 5:
77,110
78,112
80,116
81,117

Level 6:
75,120
76,120
78,124
78,124
79,126
80,128

Level 7:
75,129
76,131
77,133
78,135
81,141

Level 8:
75,141
77,145
78,148
80,153

Level 9:
76,157
77,159
78,162
80,168
80,168
80,169
84,175

Level 10:
80,178

Level 11:

74,173
75,175
76,178
77,181
78,184
79,187
79,187
80,190
80,190
82,136
Watt = 190-(rpm-80)*3

Level 12:
76,188
77,191
78,194
80,200
84,211
Watt = 200-(rpm-80)*3

Level 13:
76,198
77,201
78,204
79,207
80,210
Watt = 210-(rpm-80)*3

Level 14:
77,210
78,213
79,216
79,217
80,220
84,230
85,233
Watt = 220-(rpm-80)*3

Level 15:
72,204
73,207
74,210
75.215
80,230
82,232
82,235
85,238
Watt = 230-(rpm-80)*3

Level 16:
71,211
72,215
76,229
77,232
78,236
78,236
79,239
80,243
Watt = 236-(rpm-78)*4

Level 17:
72,229
74,233
75,237
76,240
76,240
77,244
78,247
Watt = 240+(rpm-76)*4

Level 18:
72,236
73,240
75,248
75,248
76,251
77,255
78,259
Watt = 248+(rpm-75)*4

Level 19:
74,253
75,257
76,261
77,265
Watt = 258+(rpm-75)*4

Level 20:
70,244
71,248
73,256
74,260
Watt = 260+(rpm-74)*4

Polar:
https://www.polar.com/sites/default/files/static/science/white-papers/polar-smart-calories-white-paper.pdf

Energy Expediture:
https://www.researchgate.net/publication/7777759_Prediction_of_energy_expenditure_from_heart_rate_monitoring_during_submaximal_exercise
EE = gender x (-55.0969 + 0.6309 x heart rate + 0.1988 x weight + 0.2017 x age)
+ (1 - gender) x (-20.4022 + 0.4472 x heartrate - 0.1263 x weight + 0.074 x age)
where gender = 1 for males and 0 for females. 

or:
https://www.sciencedirect.com/science/article/pii/S2095254612000464
for men, REE = 66.5 + 13.75 × weight (kg) + 5.003 × height (cm) − 6.775 × age; for women, REE = 655.1 + 9.563 × weight (kg) + 1.850 × height (cm) − 4.676 × age.

Calculator cadence:
https://www.omnicalculator.com/sports/bike-cadence-and-speed-calculator

https://www.bikecalc.com/speed_at_cadence

External buttons connection:

Candidates P0.11, P0.12, P0.22 + GND 

FTMS:
https://stackoverflow.com/questions/59653425/zwift-add-resistance-with-ftms-control-point

When an app like Zwift write to the CP it's an OP Code optionally followed by parameters, you should reply with the OP Code 0x80 (Response Code) followed by the OP Code that the reply is for, optionally followed by parameters.

In the case of OP Code 0x00 (Request Control) you should therefore reply with: 0x80, 0x00, 0x01. The last 0x01 is the result code for "Success".

In the case of OP Code 0x07 (Start/Resume), you should reply with: 0x80, 0x07, 0x01. Assuming you consider the request successful, the other possible responses are detailed in Table 4.24 of the FTMS Specification PDF.

On command you should look into is OP Code 0x11. When running Zwift in "non-workout mode", so just the normal mode, Zwift will use OP Code 0x11 followed by a set of simulation parameters: Wind Speed, Grade, Rolling resistance coeff and Wind resistance coeff. See the "4.16.2.18 Set Indoor Bike Simulation Parameters Procedure" of the PDF for details on the format of those parameters.


https://github.com/erikboto/ftms-example
https://forums.garmin.com/developer/connect-iq/f/app-ideas/206704/developing-a-ciq-ble-client-for-treadmill-and-fitness-equipment
Fitness Machine Service UUID: 0x1826
Fitness Machine Feature Characteristic 0x2acc 

Treadmill Data Characteristic 0x2acd
2902 Enable Notify Descriptor 0x2902
This is required so client gets more than one data point.
Supported Speed Range Characteristic 0x2ad4
Supported Incline Range Characteristic 0x2ad5
Keep Alive UUID


https://github.com/chadj/gpedal
https://devzone.nordicsemi.com/f/nordic-q-a/56370/bluetooth-ble-52832-try-to-implement-ftms-control-point-with-zwift
https://devzone.nordicsemi.com/f/nordic-q-a/56157/ble--nrfconnect-ftms-profile-indoor-bike-data-characteristic
https://devzone.nordicsemi.com/f/nordic-q-a/31046/how-to-support-ble-ftmp-profile
https://github.com/zacharyedwardbull/pycycling
The SufferFest, TrainerRoad, and Kinomap are the only two that I know of. 

ANT FE-C:
https://devzone.nordicsemi.com/f/nordic-q-a/44110/fe-c-support
https://github.com/vincent290587/ant_profiles

https://github.com/microsoft/BluetoothLEExplorer

https://stackoverflow.com/questions/64002583/decode-bluetooth-data-from-the-indoor-bike-data-characteristic/64004591#64004591

Tacx I/O FE-C
https://github.com/abellono/tacx-ios-bluetooth-example

https://github.com/jedla22/BleTrainerControl/blob/97f8fcefe4eb8563b768818658a1b480e10bd1b2/BleTrainerControl/BTLETrainerManager.m#L824
https://github.com/weinzmi/daumUSB2BLE/blob/master/BLE/fitness-control-point-characteristic.js#L128

Firmware download URL
https://device.tacxtraining.com/firmware/update.json

https://github.com/DigitalSecurity/nrf5x-tools

https://device.tacxtraining.com/firmware/neo_sb_pack_v0.0.40.zip

//Tacx FE-C service and characteristics
#define TACX_FEC_PRIMARY_SERVICE @"6E40FEC1-B5A3-F393-E0A9-E50E24DCCA9E"
//Pour détecter le Vortex
#define TACX_VORTEX_PRIMARY_SERVICE @"669AA305-0C08-969E-E211-86AD5062675F"


#define TACX_FEC_READ_CHARACTERISTIC @"6E40FEC2-B5A3-F393-E0A9-E50E24DCCA9E"
#define TACX_FEC_WRITE_CHARACTERISTIC @"6E40FEC3-B5A3-F393-E0A9-E50E24DCCA9E"


	
test: a4 09 4e 05 31 ff ff ff ff ff 90 01 b9 - set watt to 100W (= 400 * 0.25W)
test: a4 09 4e 05 31 ff ff ff ff ff 80 04 ac - set watt to 300w ( = 1200 * 0.25W)
test: a4 09 4e 05 33 ff ff ff ff 32 00 32 77 -  set incline to 50, roll resistance 0x32
test: a4 09 4e 05 33 ff ff ff ff 96 00 32 d3 -  set incline to 150
      a4 03 4e 05 46 47 ed - request data page 76 (last command)
	  a4 03 4e 05 46 10 ba - request type. 2nd byte in response = type of equipment (25 / 0x19 = Indoor bike)
Sync: 0xa4
Length: 0x09  (Data+Chk)
Type: 0x4e
Channel: 0x05
Data: 0x10 0x19 0x04 0x00 0x00 0x00 0xff 0x24
Chk(XOR of EVERYTHING): 0x30

Basline resistance values:
https://zwiftinsider.com/wahoo-kickr-tests-connecting-via-legacy-ant-ant-fe-c-and-bluetooth-ftms/
https://forums.zwift.com/t/baseline-resistance-too-low-in-sim-mode/497294

Trainer force curves:
http://www.powercurvesensor.com/cycling-trainer-power-curves/

https://devzone.nordicsemi.com/f/nordic-q-a/23316/nrf_error_no_mem

https://bicycles.stackexchange.com/questions/49732/does-changing-gear-relation-produce-different-speed-at-same-applied-power


Power via battery:

USB and coin cell at the same time is OK as long as protection diodes are not shorted: https://devzone.nordicsemi.com/f/nordic-q-a/44780/can-we-supply-power-from-both-usb-and-coin-cell-battery

Infos about battery:
https://www.dmcinfo.com/Portals/0/Blog%20Files/High%20pulse%20drain%20impact%20on%20CR2032%20coin%20cell%20battery%20capacity.pdf

Info about nrf51 sampling battery voltage via saadc
https://github.com/NordicPlayground/nrf51-ADC-examples/blob/master/ble_app_hrs_adc_battery_measurement/main.c

There should be an Input PIN 
https://github.com/NordicPlayground/nRF52-ADC-examples
https://github.com/NordicPlayground/nRF52-ADC-examples/blob/master/ble_app_uart__saadc_timer_driven__scan_mode/main.c

nrf_saadc_input_t NRF_SAADC_INPUT_VDD
nrf_saadc_reference_t NRF_SAADC_REFERENCE_VDD4 
battery_level_in_percent in https://infocenter.nordicsemi.com/topic/sdk_nrf5_v17.0.2/group__app__util.html

Important finding: The DK52 has protection diodes on all 3 powering options which lead to a drop of 0.5V on every option. Instead of 3.3V via USB its only 2.8V, Instead of 3V for the coin cell, its only 2.5V... My calculations for the triggering of the home trainer are based on a voltage of 3V (resistor, current, ..). The voltage of 2.5V is too low to trigger the shift! Actually, even at 2.8V it's a wonder it actually works...

 
USB A Host Interface: https://learn.sparkfun.com/tutorials/usb-type-a-female-breakout-hookup-guide/all

Central micro on hometrainer is presumably a Reneasas R8C/24, https://www.digchip.com/datasheets/parts/datasheet/192/R5F21258SNFP-pdf.php
Latest datasheet https://www.renesas.com/br/en/document/dst/r8c24-r8c25-group-datasheet?language=en&r=1053566

Hometrainer, 4 pin "Sensor" Cable:
(Note: Colors are relativ to my Y-Splitter cable!)
Yellow: GND
Red: Power settings, ranging from 1.375V (Resistance 1) to 3.9V (Resistance 32) [1: 1.375V, 10: 2.4V, 20: 3.18V, 30: 3.8V, 32: 3.9V]
White: 5V, Cadence signal, on every full revolution, signal goes Low for n ms (100?)
Black: 12V, can be used to get some voltage


