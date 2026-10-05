import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = []

ns = cg.esphome_ns.namespace("ogeneral_bridge")
OGeneralBridge = ns.class_("OGeneralBridge", cg.Component)

PIN = cv.int_range(min=0, max=30)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OGeneralBridge),
        cv.Required("a_rx"): PIN,  # LIN transceiver "A" (wall remote side) RXD -> ESP input
        cv.Required("a_tx"): PIN,  # LIN transceiver "A"                    TXD <- ESP output
        cv.Required("b_rx"): PIN,  # LIN transceiver "B" (AC indoor unit side) RXD
        cv.Required("b_tx"): PIN,  # LIN transceiver "B"                       TXD
        cv.Optional("forward", default=True): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_pins(config["a_rx"], config["a_tx"], config["b_rx"], config["b_tx"]))
    cg.add(var.set_forward(config["forward"]))
