# esphome-basen
Basengreen BMS component for ESPHome

ESPHome component to monitor a Basen Battery Management System (Basen-BMS) via RS485

Contributions and feedback are welcome to improve functionality.

I created this component to read out the BMS battery information from the Basengreen 48V DIY kit BMS as I've found no solutions that worked with my BMS version.
The BMS reports as *HY-CW007-B200LT55-V1.0.0*.

To configure the BMS, ensure that all battery packs have a unique ID set via DIP switches (ID != 0), as the ESP acts as the master on the RS485 bus.
For connection the secondary RS485 ports are used (not the CAN or RS485 to the inverter).

## Requirements

* [ESPHome 2024.6.0 or higher](https://github.com/esphome/esphome/releases).
* Generic ESP32 board
* RS485 converter on a ESP32 UART

## Installation

You can install this component with [ESPHome external components feature](https://esphome.io/components/external_components.html) like this:
```yaml
external_components:
  - source: github://GHswitt/esphome-basen@main
```

or just use the `esphome-basen-bms.yaml` and modify it as needed.

If your RS485 transceiver does not support automatic flow control and uses a pin for manual flow control
add `flow_control_pin: <PIN>` to the `basen_controller` component.

## Supported Sensor Metrics

- connected: Indicates if the BMS is currently connected (online status).
- bms_version: Reports the BMS firmware or hardware version.
- barcode: Displays the BMS barcode or serial number.
- status_bitmask: Shows the raw status bitmask as a string.
- status: Human-readable status.
- alarm: Alarm status.
- fault: Fault status.
- error_bitmask: Error bitmask created from status bitmask, see error_bitmask definition below.
- charging_enabled: True when the charge MosFET is enabled.
- discharging_enabled: True when the discharge MosFET is enabled.
- heating_status: True when the heating (pad) MosFET is enabled.

- voltage: Battery voltage (V)
- current: Battery current (A)
- power: Battery power (W)
- capacity: Battery capacity (Ah)
- state_of_charge: State of charge (%)
- state_of_health: State of health (%)
- cycles: Charge/discharge cycles (count)
- temperature1: Temperature sensor 1 (°C)
- temperature2: Temperature sensor 2 (°C)
- temperature3: Temperature sensor 3 (°C)
- temperature4: Temperature sensor 4 (°C)
- temperature_mos: MOSFET temperature (°C)
- temperature_ambient: Ambient temperature (°C)
- cell_voltage_01-cell_voltage_16: Cell voltage (V)
- avg_cell_voltage: Average cell voltage (V)
- min_cell_voltage: Minimum cell voltage (V)
- max_cell_voltage: Maximum cell voltage (V)
- min_cell_index: Index of minimum voltage cell
- max_cell_index: Index of maximum voltage cell
- delta_cell_voltage: Difference between max and min cell voltage (V)
- cell_balancing_bitmask: Bitmask of cells that are balancing

### error_bitmask
The `error_bitmask` is derived from the status bitmask for easier interpretation of error states. Each bit represents a specific alarm or error condition:

| Bit  | Description                              | Binary Mask            | Hex Mask   |
|------|------------------------------------------|------------------------|------------|
| 0    | General alarm                            | 0000 0000 0000 0001    | 0x0001     |
| 1    | Battery high voltage alarm               | 0000 0000 0000 0010    | 0x0002     |
| 2    | Battery low voltage alarm                | 0000 0000 0000 0100    | 0x0004     |
| 3    | Battery high temperature alarm           | 0000 0000 0000 1000    | 0x0008     |
| 4    | Battery low temperature alarm            | 0000 0000 0001 0000    | 0x0010     |
| 5    | Battery high temperature charge alarm    | 0000 0000 0010 0000    | 0x0020     |
| 6    | Battery low temperature charge alarm     | 0000 0000 0100 0000    | 0x0040     |
| 7    | Battery high discharge current alarm     | 0000 0000 1000 0000    | 0x0080     |
| 8    | Battery high charge current alarm        | 0000 0001 0000 0000    | 0x0100     |
| 9    | Contactor alarm                          | 0000 0010 0000 0000    | 0x0200     |
| 10   | Short circuit alarm                      | 0000 0100 0000 0000    | 0x0400     |
| 11   | BMS internal alarm                       | 0000 1000 0000 0000    | 0x0800     |
| 12   | Cell imbalance alarm                     | 0001 0000 0000 0000    | 0x1000     |
| 13   | Reserved                                 | 0010 0000 0000 0000    | 0x2000     |
| 14   | Reserved                                 | 0100 0000 0000 0000    | 0x4000     |
| 15   | Reserved                                 | 1000 0000 0000 0000    | 0x8000     |

## Status messages

The following status messages can be decoded, though many have not been tested due to practical constraints or safety considerations:

- Cell voltage low fault
- Voltage line break
- Charge MOS fault
- Dischare MOS fault
- Voltage sensor fault
- NTC disconnection
- ADC MOD fault
- Reverse battery
- Hot failure
- Battery locked
- Communication timeout
- SN code failure
- Secondary trip protection
- DISC_OV_TEMP_Protection
- DISC_UN_TEMP_Protection
- 536_COM_Timeout
- Charging
- Discharging
- Short Circuit Protection
- Overcurrent Protection
- Overvoltage Protection
- CHG_OV_TEMP_Protection
- CHG_UN_TEMP_Protection
- Ambient_Low_TEMP_Protection
- ENV_High_TEMP_Protection
- SOC Low protect
- Manual_CHG_MOS_Open
- Manual_CHG_MOS_Off
- Manual_DISC_MOS_Open
- Manual_DISC_MOS_Off
- Heating pad
- MOSFET_OV_TEMP_Protection
- MOSFET_LO_TEMP_Protection
- CHG_Open_TEMP_Too_Low
- SOC_Low_Alarm2
- Vibration Alarm
- Waiting to recharge during a break
- Aerosol fault
- SOH Low Alarm
- NTC short circuit
- Temp_Diff Alarm
- System Lock
- AMB_Over_TEMP_Alarm
- AMB_Low_TEMP_Alarm
- MOS_Over_TEMP_Alarm
- SOC_Low_Alarm
- Vol_DIF_Alarm
- BAT_DISC_Over_TEMP_Alarm
- BAT_DISC_Low_TEMP_Alarm
- Cell_Over_VOL_Alarm
- Cell_Low_VOL_Alarm
- Pack_Over_VOL_Alarm
- Pack_Low_VOL_Alarm
- CHG_Over_CUR_Alarm
- DISC_Over_CURR_Alarm
- BAT_CHG_Over_TEMP_Alarm
- BAT_CHG_Low_TEMP_Alarm

## Alarm Parameters

The following alarm parameters are read (typically once during startup):

- CELL_OV_Start_Alarm
- CELL_OV_Delay_Alarm
- CELL_OV_Stop_Alarm
- CELL_UV_Start_Alarm
- CELL_UV_Delay_Alarm
- CELL_UV_Stop_Alarm
- PACK_OV_Start_Alarm
- PACK_OV_Delay_Alarm
- PACK_OV_Stop_Alarm
- PACK_UV_Start_Alarm
- PACK_UV_Delay_Alarm
- PACK_UV_Stop_Alarm
- CHG_OC_Start_Alarm
- CHG_OC_Delay_Alarm
- CHG_OC_Stop_Alarm
- DISC_OC_Start_Alarm
- DISC_OC_Delay_Alarm
- DISC_OC_Stop_Alarm
- CHG_OT_START_Alarm
- CHG_OT_Delay_Alarm
- CHG_OT_STOP_Alarm
- CHG_UT_START_Alarm
- CHG_UT_Delay_Alarm
- CHG_UT_STOP_Alarm
- DISC_OT_START_Alarm
- DISC_OT_Delay_Alarm
- DISC_OT_STOP_Alarm
- DISC_UT_START_Alarm
- DISC_UT_Delay_Alarm
- DISC_UT_STOP_Alarm
- MOS_OT_START_Alarm
- MOS_OT_Delay_Alarm
- MOS_OT_STOP_Alarm
- Capacity_Low_Start_Alarm
- Capacity_Low_Stop_Alarm
- Vol_Diff_Start_Alarm
- Vol_Diff_Stop_Alarm
- ENV_OT_START_Alarm
- ENV_OT_Delay_Alarm
- ENV_OT_STOP_Alarm
- ENV_UT_START_Alarm
- ENV_UT_Delay_Alarm
- ENV_UT_STOP_Alarm

These parameters define the thresholds and delays for various alarm conditions reported by the BMS.

## Protection Parameters

Additionally the following protection parameters are read (one time during startup):

- CELL_OV_Start
- CELL_OV_Delay
- CELL_OV_Stop
- CELL_UV_Start
- CELL_UV_Delay
- CELL_UV_Stop
- PACK_OV_Start
- PACK_OV_Delay
- PACK_OV_Stop
- PACK_UV_Start
- PACK_UV_Delay
- PACK_UV_Stop
- Const_Pack_V
- Const_Current
- CHG_OC1_Start
- CHG_OC1_Delay
- DISC_OC1_Start
- DISC_OC1_Delay
- CHG_OC2_Start
- CHG_OC2_Delay
- DISC_OC2_Start
- DISC_OC2_Delay
- CHG_OT_START
- CHG_OT_Delay
- CHG_OT_STOP
- DISC_OT_START
- DISC_OT_Delay
- DISC_OT_STOP
- CHG_UT_START
- CHG_UT_Delay
- CHG_UT_STOP
- DISC_UT_START
- DISC_UT_Delay
- DISC_UT_STOP
- MOS_OT_START
- MOS_OT_Delay
- MOS_OT_STOP
- ENV_OT_START
- ENV_OT_Delay
- ENV_OT_STOP
- ENV_UT_START
- ENV_UT_Delay
- ENV_UT_STOP
- Balance_Start_Vol
- Balance_Start_Diff
- Sleep_Cell_Volt
- Shorts_Delay
- Standby_Time
- UV_OFF_Time
- LC_Style
- Sleep_Time

## BMS Serial Protocol Description

The BMS uses a custom UART serial protocol for communication. Below is a summary of the protocol structure and behavior:

### Frame Structure

Each frame (command or response) consists of the following bytes:

| Byte(s) | Description                                      |
|---------|--------------------------------------------------|
| 0       | SOI (Start of Information), always `0x7E`        |
| 1       | Address (device address, usually `0x01`)         |
| 2       | Command code (e.g., `0x01` for info, `0x33` for version) |
| 3       | Secondary command code or for response data length (number of data bytes to follow)     |
| 4..N    | Data payload (length as specified above)         |
| N+1     | Checksum (see below)                             |
| N+2     | EOI (End of Information), always `0x0D`          |

### Checksum Calculation

- The checksum is calculated over all bytes from SOI up to the last data byte (not including the checksum or EOI).
- Calculation:  
  - `num1` is the XOR of all bytes.
  - `num2` is the sum of all bytes.
  - The checksum byte is `(num1 ^ num2) & 0xFF`.

### Command Codes

- `0x01`: Info (cell voltages, SoC, SoH, etc.)
- `0x33`: BMS Version
- `0x42`: Barcode
- `0x43 0xE0`: Protection parameters

### Example Command Frame

To request info from the BMS at address 0x01:

| Byte(s) | Description              |
|---------|--------------------------|
| 0       | SOI (Start of Information), always `0x7E` |
| 1       | Address (device address, usually `0x01`) |
| 2       | Command code (e.g., `0x01` for info) |
| 3       | Length (number of data bytes to follow) |
| 4       | Checksum (calculated as described above) |
| 5       | EOI (End of Information), always `0x0D` |

```
    7e 01 01 00 fe 0d                                 ~...þ.           
```

### Example Response Frame

A typical response might look like:

```
    7e 01 01 7c 01 10 0d 44 0d 41 0d 40 0d 41 0d 40   ~..|...D.A.@.A.@ 
    0d 3f 0d 40 0d 41 0d 43 0d 42 0d 42 0d 42 0d 43   .?.@.A.C.B.B.B.C 
    0d 42 0d 42 0d 45 02 01 75 30 03 01 27 0f 04 01   .B.B.E..u0..'... 
    6d 60 05 06 00 47 00 48 00 48 00 47 40 48 20 4b   m`...G.H.H.G@H K 
    06 05 00 00 00 00 00 00 00 00 00 00 07 01 00 1c   ................ 
    08 01 15 35 09 01 27 10 0a 01 00 00 0b 01 00 00   ...5..'......... 
    1a 05 0c 01 00 00 19 5d 0d 01 00 2b 27 ef 0e 01   .......]...+'ï.. 
    00 40 00 00 0f 01 00 05 75 06 10 01 00 04 4b a2   .@......u.....K¢ 
    62 0d                                             b.    
```

### Notes

- All multi-byte values are big-endian.
- The protocol supports multiple devices on the same bus by address.

## Debugging

If this component doesn't work out of the box for your device please update your configuration to increase the log level to see details:

```
logger:
  level: VERY_VERBOSE
  logs:
    basen_controller: VERY_VERBOSE
    basen_bms: VERY_VERBOSE
    scheduler: DEBUG
    component: DEBUG
    sensor: DEBUG
    mqtt: INFO
    mqtt.idf: INFO
    mqtt.component: INFO
    mqtt.sensor: INFO
    mqtt.switch: INFO
    api.service: INFO
    api: INFO
```