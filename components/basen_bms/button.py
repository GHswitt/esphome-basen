import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import (
    CONF_ID, ENTITY_CATEGORY_CONFIG, ICON_EMPTY, CONF_ICON
)

from . import CONF_BASEN_BMS_ID, BasenBMS, basen_ns

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_HEATING_ON = "heating_on"
CONF_HEATING_OFF = "heating_off"

BUTTONS = {
    CONF_HEATING_ON     : [0xE3, 0x01],
    CONF_HEATING_OFF    : [0xE3, 0x02],
}

BasenButton = basen_ns.class_("BasenButton", button.Button, cg.Component)

BASEN_BUTTON_SCHEMA = (
    button.button_schema(
        BasenButton,
        icon=ICON_EMPTY,
        entity_category=ENTITY_CATEGORY_CONFIG,
    )
    .extend(cv.COMPONENT_SCHEMA)
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        cv.Optional(CONF_HEATING_ON): BASEN_BUTTON_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:radiator"): cv.icon,
            }
        ),
        cv.Optional(CONF_HEATING_OFF): BASEN_BUTTON_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:radiator-off"): cv.icon,
            }
        ),
    }
)


# def to_code(config):
#     hub = yield cg.get_variable(config[CONF_BASEN_BMS_ID])
#     for key, command in BUTTONS.items():
#         if key in config:
#             conf = config[key]
#             var = cg.new_Pvariable(conf[CONF_ID])
#             yield button.register_button(var, conf)
#             cg.add(getattr(hub, f"set_{key}_button")(var))
#             cg.add(var.set_parent(hub))
#             cg.add(var.set_type(command[0], command[1]))
