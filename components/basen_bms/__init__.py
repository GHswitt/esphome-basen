import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import basen_controller
from esphome.const import CONF_ID, CONF_THROTTLE

from ..basen_controller import CONF_BASEN_CONTROLLER_ID

AUTO_LOAD = ["basen_controller", "binary_sensor", "sensor", "text_sensor", "number", "button", "switch"]

#DEPENDENCIES = ["basen_controller"]

CODEOWNERS = ["GHswitt"]

MULTI_CONF = True

basen_ns = cg.esphome_ns.namespace("basen")
BasenBMS = basen_ns.class_("BasenBMS", cg.PollingComponent)

CONF_BASEN_BMS_ID = "basen_bms_id"
CONF_ADDRESS = "address"

CONFIG_SCHEMA = (
    cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BasenBMS),
        cv.Required(CONF_ADDRESS): cv.int_,
    }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(basen_controller.BasenBMS_schema())
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_address(config[CONF_ADDRESS]))
    hub = await cg.get_variable(config[CONF_BASEN_CONTROLLER_ID])
    cg.add(var.set_parent(hub))

