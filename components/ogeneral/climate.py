import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart

DEPENDENCIES = ["uart"]

ogeneral_ns = cg.esphome_ns.namespace("ogeneral")
OGeneralClimate = ogeneral_ns.class_("OGeneralClimate", climate.Climate, cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = (
    climate.climate_schema(OGeneralClimate)
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
