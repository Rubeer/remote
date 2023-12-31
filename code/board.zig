const std = @import("std");
const gpio = @import("gpio.zig");

pub const regs = @import("STM32G030.zig").devices.STM32G030.peripherals;

const GPIOA = gpio.GPIOA;
const GPIOB = gpio.GPIOB;
const GPIOC = gpio.GPIOC;
const GPIOD = gpio.GPIOD;

pub const pins = .{
    .row1 = "PA1",
    .row2 = "PA3",
    .row3 = "PA5",
    .row4 = "PC15",
    .col1 = "PA0",
    .col2 = "PA2",
    .col3 = "PA4",

    .ir_in_enable = "PA12",
    .ir_in = "PA8",

    .ir_out = "PB9",
    .ir_out_boost = "PB7",

    .encoder_push = "PA11",
    .encoder_wkup = "PB6",
    .encoder_a = "PA6",
    .encoder_b = "PA7",
};

const Cfg = gpio.Config;

pub const startup_pin_config = .{
    Cfg{ .name = pins.row1, .mode = .output, .level = .low },
    Cfg{ .name = pins.row2, .mode = .output, .level = .low },
    Cfg{ .name = pins.row3, .mode = .output, .level = .low },
    Cfg{ .name = pins.row4, .mode = .output, .level = .low },
    Cfg{ .name = pins.col1, .mode = .input, .pull = .up, .output_type = .open_drain },
    Cfg{ .name = pins.col2, .mode = .input, .pull = .up, .output_type = .open_drain },
    Cfg{ .name = pins.col3, .mode = .input, .pull = .up, .output_type = .open_drain },

    Cfg{ .name = pins.ir_in_enable, .mode = .output, .level = .low },
    Cfg{ .name = pins.ir_in, .mode = .analog },

    Cfg{ .name = pins.ir_out, .mode = .output, .alt_mode = .AF0, .output_type = .push_pull, .level = .low },
    Cfg{ .name = pins.ir_out_boost, .mode = .output, .level = .low },

    Cfg{ .name = pins.encoder_push, .mode = .input, .pull = .up },
    Cfg{ .name = pins.encoder_wkup, .pull = .up },
    Cfg{ .name = pins.encoder_a, .mode = .alternate, .alt_mode = .AF1, .pull = .up },
    Cfg{ .name = pins.encoder_b, .mode = .alternate, .alt_mode = .AF1, .pull = .up },
};

pub const cols_as_input = .{
    Cfg{ .name = pins.col1, .mode = .input, .pull = .up },
    Cfg{ .name = pins.col2, .mode = .input, .pull = .up },
    Cfg{ .name = pins.col3, .mode = .input, .pull = .up },
};

pub const cols_as_opendrain_highz = .{
    Cfg{ .name = pins.col1, .mode = .output, .pull = .none, .level = .high },
    Cfg{ .name = pins.col2, .mode = .output, .pull = .none, .level = .high },
    Cfg{ .name = pins.col3, .mode = .output, .pull = .none, .level = .high },
};

pub const rows_low = .{
    Cfg{ .name = pins.row1, .level = .low },
    Cfg{ .name = pins.row2, .level = .low },
    Cfg{ .name = pins.row3, .level = .low },
    Cfg{ .name = pins.row4, .level = .low },
};
pub const rows_high = .{
    Cfg{ .name = pins.row1, .level = .high },
    Cfg{ .name = pins.row2, .level = .high },
    Cfg{ .name = pins.row3, .level = .high },
    Cfg{ .name = pins.row4, .level = .high },
};

pub const ir_output_off = .{
    Cfg{ .name = pins.ir_out, .mode = .output, .output_type = .push_pull, .level = .low },
};
pub const ir_output_on = .{
    Cfg{ .name = pins.ir_out, .mode = .alternate },
};
