import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_BATTERY_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ICON_EMPTY,
    ICON_FLASH,
    ICON_GAUGE,
    ICON_PERCENT,
    ICON_THERMOMETER,
    ICON_BATTERY,
    ICON_COUNTER,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_EMPTY,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
    UNIT_MINUTE,
    UNIT_MILLISECOND
)

from . import CONF_BASEN_BMS_ID, BasenBMS

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_VOLTAGE = "voltage"
CONF_CURRENT = "current"
CONF_POWER = "power"
CONF_TEMPERATURE1 = "temperature1"
CONF_TEMPERATURE2 = "temperature2"
CONF_TEMPERATURE3 = "temperature3"
CONF_TEMPERATURE4 = "temperature4"
CONF_TEMPERATURE_MOS = "temperature_mos"
CONF_TEMPERATURE_AMBIENT = "temperature_ambient"
CONF_CAPACITY = "capacity"
CONF_STATE_OF_CHARGE = "state_of_charge"
CONF_STATE_OF_HEALTH = "state_of_health"
CONF_CYCLES = "cycles"
CONF_AVG_CELL_VOLTAGE = "avg_cell_voltage"
CONF_MIN_CELL_VOLTAGE = "min_cell_voltage"
CONF_MAX_CELL_VOLTAGE = "max_cell_voltage"
CONF_DELTA_CELL_VOLTAGE = "delta_cell_voltage"
CONF_MIN_CELL_INDEX = "min_cell_index"
CONF_MAX_CELL_INDEX = "max_cell_index"
CONF_ERROR_BITMASK = "error_bitmask"
UNIT_AMPERE_HOUR = "Ah"

SENSORS = [
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_POWER,
    CONF_CAPACITY,
    CONF_STATE_OF_CHARGE,
    CONF_STATE_OF_HEALTH,
    CONF_CYCLES,
    CONF_TEMPERATURE1,
    CONF_TEMPERATURE2,
    CONF_TEMPERATURE3,
    CONF_TEMPERATURE4,
    CONF_TEMPERATURE_MOS,
    CONF_TEMPERATURE_AMBIENT,
    CONF_AVG_CELL_VOLTAGE,
    CONF_MIN_CELL_VOLTAGE,
    CONF_MAX_CELL_VOLTAGE,
    CONF_MIN_CELL_INDEX,
    CONF_MAX_CELL_INDEX,
    CONF_DELTA_CELL_VOLTAGE,
    CONF_ERROR_BITMASK
]

# The index matches the type of the parameter as received from the device.
PARAMETERS = [
    "CELL_OV_Start",            # 0x00
    "CELL_OV_Delay",            # 0x01
    "CELL_OV_Stop",             # 0x02
    "CELL_UV_Start",            # 0x03
    "CELL_UV_Delay",            # 0x04
    "CELL_UV_Stop",             # 0x05
    "PACK_OV_Start",            # 0x06
    "PACK_OV_Delay",            # 0x07
    "PACK_OV_Stop",             # 0x08
    "PACK_UV_Start",            # 0x09
    "PACK_UV_Delay",            # 0x0A
    "PACK_UV_Stop",             # 0x0B
    "Const_Pack_V",             # 0x0C
    "Const_Current",            # 0x0D
    "CHG_OC1_Start",            # 0x0E
    "CHG_OC1_Delay",            # 0x0F
    "DISC_OC1_Start",           # 0x10
    "DISC_OC1_Delay",           # 0x11
    "CHG_OC2_Start",            # 0x12
    "CHG_OC2_Delay",            # 0x13
    "DISC_OC2_Start",           # 0x14
    "DISC_OC2_Delay",           # 0x15
    "CHG_OT_START",             # 0x16
    "CHG_OT_Delay",             # 0x17
    "CHG_OT_STOP",              # 0x18
    "DISC_OT_START",            # 0x19
    "DISC_OT_Delay",            # 0x1A
    "DISC_OT_STOP",             # 0x1B
    "CHG_UT_START",             # 0x1C
    "CHG_UT_Delay",             # 0x1D
    "CHG_UT_STOP",              # 0x1E
    "DISC_UT_START",            # 0x1F
    "DISC_UT_Delay",            # 0x20
    "DISC_UT_STOP",             # 0x21
    "MOS_OT_START",             # 0x22
    "MOS_OT_Delay",             # 0x23
    "MOS_OT_STOP",              # 0x24
    "ENV_OT_START",             # 0x25
    "ENV_OT_Delay",             # 0x26
    "ENV_OT_STOP",              # 0x27
    "ENV_UT_START",             # 0x28
    "ENV_UT_Delay",             # 0x29
    "ENV_UT_STOP",              # 0x2A
    "Balance_Start_Vol",        # 0x2B
    "Balance_Start_Diff",       # 0x2C
    "Sleep_Cell_Volt",          # 0x2D
    "Shorts_Delay",             # 0x2E
    "Standby_Time",             # 0x2F
    "UV_OFF_Time",              # 0x30
    "LC_Style",                 # 0x31
    "Sleep_Time",               # 0x32
]

ALARM_PARAMETERS = [
    "CELL_OV_Start_Alarm",       # 0x00
    "CELL_OV_Delay_Alarm",       # 0x01
    "CELL_OV_Stop_Alarm",        # 0x02
    "CELL_UV_Start_Alarm",       # 0x03
    "CELL_UV_Delay_Alarm",       # 0x04
    "CELL_UV_Stop_Alarm",        # 0x05
    "PACK_OV_Start_Alarm",       # 0x06
    "PACK_OV_Delay_Alarm",       # 0x07
    "PACK_OV_Stop_Alarm",        # 0x08
    "PACK_UV_Start_Alarm",       # 0x09
    "PACK_UV_Delay_Alarm",       # 0x0A
    "PACK_UV_Stop_Alarm",        # 0x0B
    "CHG_OC_Start_Alarm",        # 0x0C
    "CHG_OC_Delay_Alarm",        # 0x0D
    "CHG_OC_Stop_Alarm",         # 0x0E
    "DISC_OC_Start_Alarm",       # 0x0F
    "DISC_OC_Delay_Alarm",       # 0x10
    "DISC_OC_Stop_Alarm",        # 0x11
    "CHG_OT_START_Alarm",        # 0x12
    "CHG_OT_Delay_Alarm",        # 0x13
    "CHG_OT_STOP_Alarm",         # 0x14
    "CHG_UT_START_Alarm",        # 0x15
    "CHG_UT_Delay_Alarm",        # 0x16
    "CHG_UT_STOP_Alarm",         # 0x17
    "DISC_OT_START_Alarm",       # 0x18
    "DISC_OT_Delay_Alarm",       # 0x19
    "DISC_OT_STOP_Alarm",        # 0x1A
    "DISC_UT_START_Alarm",       # 0x1B
    "DISC_UT_Delay_Alarm",       # 0x1C
    "DISC_UT_STOP_Alarm",        # 0x1D
    "MOS_OT_START_Alarm",        # 0x1E
    "MOS_OT_Delay_Alarm",        # 0x1F
    "MOS_OT_STOP_Alarm",         # 0x20
    "Capacity_Low_Start_Alarm",  # 0x21
    "Capacity_Low_Stop_Alarm",   # 0x22
    "Vol_Diff_Start_Alarm",      # 0x23
    "Vol_Diff_Stop_Alarm",       # 0x24
    "ENV_OT_START_Alarm",        # 0x25
    "ENV_OT_Delay_Alarm",        # 0x26
    "ENV_OT_STOP_Alarm",         # 0x27
    "ENV_UT_START_Alarm",        # 0x28
    "ENV_UT_Delay_Alarm",        # 0x29
    "ENV_UT_STOP_Alarm",         # 0x2A
]


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon="mdi:alpha-v",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            icon=ICON_FLASH,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            icon="mdi:current-dc",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_CAPACITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE_HOUR,
            icon=ICON_GAUGE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_STATE_OF_CHARGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_BATTERY,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_STATE_OF_HEALTH): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_CYCLES): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_COUNTER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_TEMPERATURE1): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE2): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE3): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE4): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE_MOS): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE_AMBIENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_AVG_CELL_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon="mdi:format-align-middle",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_MIN_CELL_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon="mdi:arrow-collapse-down",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_MAX_CELL_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon="mdi:arrow-collapse-up",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_DELTA_CELL_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon="mdi:delta",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_MIN_CELL_INDEX): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_COUNTER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_MAX_CELL_INDEX): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_COUNTER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_ERROR_BITMASK): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon="mdi:alert",
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY
        ),
    }
)

# Cell voltages
for i in range(1, 17):
    CONFIG_SCHEMA = CONFIG_SCHEMA.extend({
        cv.Optional(f"cell_voltage_{i:02}"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon="mdi:alpha-v",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        )
    })

def add_param_sensor(schema, param):
    if "Delay" in param:
        return schema.extend({
            cv.Optional(param): sensor.sensor_schema(
                unit_of_measurement=UNIT_MILLISECOND,
                icon="mdi:timer-sand",
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_EMPTY
            )
        })
    elif "Time" in param:
        return schema.extend({
            cv.Optional(param): sensor.sensor_schema(
                unit_of_measurement=UNIT_MINUTE,
                icon="mdi:timer-sand",
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_EMPTY
            )
        })
    elif "OT" in param or "UT" in param:
        return schema.extend({
            cv.Optional(param): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE
            )
        })
    elif "OC" in param or "Current" in param:
        return schema.extend({
            cv.Optional(param): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                icon="mdi:current-dc",
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CURRENT
            )
        })
    elif "_Diff" in param or "V" in param:
        return schema.extend({
            cv.Optional(param): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                icon=ICON_GAUGE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE
            )
        })
    elif "Capacity_Low" in param:
        return schema.extend({
            cv.Optional(param): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon=ICON_GAUGE,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_EMPTY
            )
        })
    else:
        return schema.extend({
            cv.Optional(param): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                icon=ICON_COUNTER,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_EMPTY
            )
        })

for param in PARAMETERS:
    CONFIG_SCHEMA = add_param_sensor(CONFIG_SCHEMA, param)

for param in ALARM_PARAMETERS:
    CONFIG_SCHEMA = add_param_sensor(CONFIG_SCHEMA, param)

def to_code(config):
    hub = yield cg.get_variable(config[CONF_BASEN_BMS_ID])
    for key in SENSORS:
        if key in config:
            conf = config[key]
            sens = yield sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
    for i in range(1, 17):
        key = f"cell_voltage_{i:02}"
        if key in config:
            conf = config[key]
            sens = yield sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_cell_voltage_sensor")(sens, i-1))
    for index, key in enumerate(PARAMETERS):
        if key in config:
            conf = config[key]
            sens = yield sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_param_sensor")(sens, index))
    for index, key in enumerate(ALARM_PARAMETERS):
        if key in config:
            conf = config[key]
            sens = yield sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_alarm_param_sensor")(sens, index))
