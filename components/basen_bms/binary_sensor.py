import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ICON, CONF_ID, ICON_SIGNAL

from . import CONF_BASEN_BMS_ID, BasenBMS

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_CONNECTED = "connected"
CONF_ALARM = "alarm"
CONF_FAULT = "fault"
CONF_CHARGING_ENABLED = "charging_enabled"
CONF_DISCHARGING_ENABLED = "discharging_enabled"
CONF_HEATING_STATUS = "heating_status"

BINARY_SENSORS = [
    CONF_CONNECTED,
    CONF_ALARM,
    CONF_FAULT,
    CONF_CHARGING_ENABLED,
    CONF_DISCHARGING_ENABLED,
    CONF_HEATING_STATUS,
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            icon=ICON_SIGNAL
        ),
        cv.Optional(CONF_ALARM): binary_sensor.binary_sensor_schema(
            icon="mdi:battery-alert"
        ),
        cv.Optional(CONF_FAULT): binary_sensor.binary_sensor_schema(
            icon="mdi:alert"
        ),
        cv.Optional(CONF_CHARGING_ENABLED): binary_sensor.binary_sensor_schema(
            icon="mdi:battery-arrow-up"
        ),
        cv.Optional(CONF_DISCHARGING_ENABLED): binary_sensor.binary_sensor_schema(
            icon="mdi:battery-arrow-down"
        ),
        cv.Optional(CONF_HEATING_STATUS): binary_sensor.binary_sensor_schema(
            icon="mdi:radiator"
        ),
    }
)


def to_code(config):
    hub = yield cg.get_variable(config[CONF_BASEN_BMS_ID])
    for key in BINARY_SENSORS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            yield binary_sensor.register_binary_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))
