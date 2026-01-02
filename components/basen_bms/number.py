import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID, UNIT_CELSIUS, ENTITY_CATEGORY_CONFIG, CONF_MODE, ICON_EMPTY,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_MODE,
    CONF_STEP,
    CONF_ICON
)

from . import CONF_BASEN_BMS_ID, BasenBMS, basen_ns

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_HEATING_ON_TEMPERATURE = "heating_on_temperature"
CONF_HEATING_OFF_TEMPERATURE = "heating_off_temperature"

NUMBERS = {
    CONF_HEATING_ON_TEMPERATURE     : 0xF2,
    CONF_HEATING_OFF_TEMPERATURE    : 0xF4
}

BasenNumber = basen_ns.class_("BasenNumber", number.Number, cg.Component)

BASEN_NUMBER_SCHEMA = (
    number.number_schema(
        BasenNumber,
        icon=ICON_EMPTY,
        entity_category=ENTITY_CATEGORY_CONFIG,
        unit_of_measurement=UNIT_CELSIUS,
    )
    .extend(
        {
            cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        cv.Optional(CONF_HEATING_ON_TEMPERATURE): BASEN_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=-20): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=30): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.float_,
                cv.Optional(CONF_ICON, default="mdi:radiator"): cv.icon,
            }
        ),
        cv.Optional(CONF_HEATING_OFF_TEMPERATURE): BASEN_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=-20): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=30): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.float_,
                cv.Optional(CONF_ICON, default="mdi:radiator-off"): cv.icon,
            }
        ),
    }
)


def to_code(config):
    hub = yield cg.get_variable(config[CONF_BASEN_BMS_ID])
    for key, command in NUMBERS.items():
        if key in config:
            conf = config[key]
            var = cg.new_Pvariable(conf[CONF_ID])
            yield number.register_number(
                var,
                conf,
                min_value=conf[CONF_MIN_VALUE],
                max_value=conf[CONF_MAX_VALUE],
                step=conf[CONF_STEP],
            )
            cg.add(getattr(hub, f"set_{key}_number")(var))
            cg.add(var.set_parent(hub))
            cg.add(var.set_type(command))
