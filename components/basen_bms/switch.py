import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import (
    CONF_ID, ICON_EMPTY, CONF_ICON
)

from . import CONF_BASEN_BMS_ID, BasenBMS, basen_ns

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_HEATING = "heating"

SWITCHES = {
    CONF_HEATING     : [0xE3, 0x01, 0x02],
}

BasenSwitch = basen_ns.class_("BasenSwitch", switch.Switch, cg.Component)

BASEN_BUTTON_SCHEMA = (
    switch.switch_schema(
        BasenSwitch,
        icon=ICON_EMPTY,
        #entity_category=ENTITY_CATEGORY_CONFIG,
    )
    .extend(cv.COMPONENT_SCHEMA)
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        cv.Optional(CONF_HEATING): BASEN_BUTTON_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:radiator"): cv.icon,
            }
        ),
    }
)


def to_code(config):
    hub = yield cg.get_variable(config[CONF_BASEN_BMS_ID])
    for key, command in SWITCHES.items():
        if key in config:
            conf = config[key]
            var = cg.new_Pvariable(conf[CONF_ID])
            yield switch.register_switch(var, conf)
            cg.add(getattr(hub, f"set_{key}_switch")(var))
            cg.add(var.set_parent(hub))
            cg.add(var.set_type(command[0], command[1], command[2]))
