import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG

from . import CONF_BASEN_BMS_ID, BasenBMS, basen_ns

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_CALIBRATION = "calibration"

# Must match the order of the CALIBRATION_ enum in basen.h
CALIBRATION_OPTIONS = [
    "None",
    "Current zero point",
    "Current charge point 1 (mA, ≤5000)",
    "Current charge point 2 (mA, >5000)",
    "Current discharge point 1 (mA, ≤5000)",
    "Current discharge point 2 (mA, >5000)",
    "Reset current charge point 1",
    "Reset current charge point 2",
    "Reset current discharge point 1",
    "Reset current discharge point 2",
]

SELECTS = {
    CONF_CALIBRATION: CALIBRATION_OPTIONS,
}

BasenSelect = basen_ns.class_("BasenSelect", select.Select, cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        cv.Optional(CONF_CALIBRATION): select.select_schema(
            BasenSelect,
            icon="mdi:tune",
            entity_category=ENTITY_CATEGORY_CONFIG,
        ).extend(cv.COMPONENT_SCHEMA),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BASEN_BMS_ID])
    for key, options in SELECTS.items():
        if key in config:
            conf = config[key]
            var = cg.new_Pvariable(conf[CONF_ID])
            await cg.register_component(var, conf)
            await select.register_select(var, conf, options=options)
            cg.add(getattr(hub, f"set_{key}_select")(var))
            cg.add(var.set_parent(hub))
