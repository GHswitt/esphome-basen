import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import CONF_ID, CONF_THROTTLE, CONF_FLOW_CONTROL_PIN
from esphome.cpp_helpers import gpio_pin_expression

DEPENDENCIES = ["uart"]

CODEOWNERS = ["GHswitt"]

MULTI_CONF = True

basen_ns = cg.esphome_ns.namespace("basen")
BasenController = basen_ns.class_("BasenController", uart.UARTDevice, cg.Component)

CONF_BASEN_CONTROLLER_ID = "basen_id"

CONFIG_SCHEMA = uart.UART_DEVICE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(BasenController),
        cv.Optional(CONF_THROTTLE, default="1s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
    }
)

def BasenBMS_schema():
    schema = {
        cv.GenerateID(CONF_BASEN_CONTROLLER_ID): cv.use_id(BasenController),
    }
    return cv.Schema(schema)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_throttle(config[CONF_THROTTLE]))
    if CONF_FLOW_CONTROL_PIN in config:
        pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))
    
