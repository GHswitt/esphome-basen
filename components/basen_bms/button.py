import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG

from . import CONF_BASEN_BMS_ID, BasenBMS, basen_ns

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_CALIBRATION = "calibration"

BasenButton = basen_ns.class_("BasenButton", button.Button, cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        # Executes the function selected with the calibration select
        cv.Optional(CONF_CALIBRATION): button.button_schema(
            BasenButton,
            icon="mdi:tune-vertical",
            entity_category=ENTITY_CATEGORY_CONFIG,
        ).extend(cv.COMPONENT_SCHEMA),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BASEN_BMS_ID])
    if CONF_CALIBRATION in config:
        conf = config[CONF_CALIBRATION]
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await button.register_button(var, conf)
        cg.add(var.set_parent(hub))
