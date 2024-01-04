const std = @import("std");
const gpio = @import("gpio.zig");
const board = @import("board.zig");
const util = @import("util.zig");
const rtt = @import("rtt.zig");
const hw = @import("hw.zig");
const ir = @import("ir.zig");

const TIM14 = board.regs.TIM14;
const GPIOA = gpio.GPIOA;

pub const rows = [_]gpio.Pin{
    gpio.parse_pin(board.pins.row1),
    gpio.parse_pin(board.pins.row2),
    gpio.parse_pin(board.pins.row3),
    gpio.parse_pin(board.pins.row4),
};

pub const columns = [_]gpio.Pin{
    gpio.parse_pin(board.pins.col1),
    gpio.parse_pin(board.pins.col2),
    gpio.parse_pin(board.pins.col3),
};

pub var matrix: [rows.len][columns.len]MatrixNode = undefined;

const LedTiming = struct {
    matrix_lookup: [10]EncodedNode,
    on_time: [10]u16,
    off_time: u16,
    at: u8,
    num: u8,
};

var g_active_timing: LedTiming = undefined;

pub const MatrixNode = struct {
    brightness: f32 = 0,
    input_timestamp: u32 = 0,
    input_state: bool = 0,
};

const EncodedNode = struct {
    row: u4,
    col: u4,

    pub fn encode(row_index: usize, column_index: usize) EncodedNode {
        return .{
            .row = @truncate(row_index),
            .col = @truncate(column_index),
        };
    }

    fn decode(self: EncodedNode) *MatrixNode {
        return &matrix[self.row][self.col];
    }
};

pub fn led_setup() void {
    var timing: LedTiming = undefined;
    timing.num = 0;
    timing.at = 0;
    var total_brightness: f32 = 0;

    for (0..rows.len) |row_index| {
        for (0..columns.len) |column_index| {
            if (row_index == 3 and (column_index == 1 or column_index == 2)) {
                continue;
            }

            const node = &matrix[row_index][column_index];

            if (node.brightness > 0) {
                total_brightness += node.brightness * node.brightness;
                timing.matrix_lookup[timing.num] = EncodedNode.encode(row_index, column_index);
                timing.num += 1;
            }
        }
    }

    const total_budget = 20000;
    const total_budget_float: f32 = @floatFromInt(total_budget);

    var led_factor: f32 = 1;
    if (total_brightness > 10) {
        led_factor = 10.0 / total_brightness;
    }

    const budget_per_led: f32 = led_factor * (total_budget_float * 0.1);
    var budget_used: u32 = 0;

    for (0..timing.num) |i| {
        const l = timing.matrix_lookup[i];
        const node = l.decode();

        const on_time: u16 = @intFromFloat(node.brightness * node.brightness * budget_per_led);
        timing.on_time[i] = on_time;
        budget_used += on_time;
    }

    if (total_budget > budget_used) {
        timing.off_time = @truncate(total_budget - budget_used);
    } else {
        timing.off_time = 0;
    }

    util.disable_irqs(&.{.TIM14});
    g_active_timing = timing;
    util.enable_irqs(&.{.TIM14});
}

fn handle_key_change(row: u8, col: u8, pressed: bool) void {
    matrix[row][col].input_state = pressed;

    if (pressed) {
        if (!ir.is_transmit_busy()) {
            if (row == 0 and col == 1) {
                ir.transmit(&ir.panasonic_volume_up);
            }
            if (row == 1 and col == 1) {
                ir.transmit(&ir.panasonic_volume_down);
            }
        }
    }

    //if (pressed) matrix[row][col].brightness = 1;
    //led_setup();
    rtt.println("Change: {},{} = {}", .{ row, col, pressed });
}

var last_scanout_ticks: u32 = 0;
var current_scanout: ?[rows.len]u8 = null;
var previous_scanout = [1]u8{0xFF} ** rows.len;

pub fn make_cols_mask() u8 {
    var mask: u32 = 0;
    inline for (columns) |col| {
        mask |= @as(u32, 1) << col.pin;
    }
    return @truncate(mask);
}

pub fn scanout_update() void {
    util.disable_irqs(&.{.TIM14});
    const new_scanout = current_scanout;
    current_scanout = null;
    util.enable_irqs(&.{.TIM14});

    if (new_scanout) |scanout| {
        const cols_mask = comptime make_cols_mask();
        for (scanout, 0..) |bits, i| {
            const diff = bits ^ previous_scanout[i];
            if (diff & cols_mask != 0) {
                for (columns, 0..) |col, j| {
                    const bit = @as(u32, 1) << col.pin;
                    if (diff & bit != 0) {
                        handle_key_change(@truncate(i), @truncate(j), bits & bit == 0);
                    }
                }
                previous_scanout[i] = bits;
            }
        }
    }
}

fn do_key_scanout() [rows.len]u8 {
    var result: [rows.len]u8 = undefined;
    last_scanout_ticks = 0;

    for (rows, &result) |row, *result_row| {
        row.set_low();

        // Adequate delay with 16MHz clock
        util.nops(15);

        result_row.* = @truncate(GPIOA.IDR.read_raw());

        row.set_high();
    }

    return result;
}

pub fn schedule_next_interrupt(time_us: u16) void {
    TIM14.ARR.write_raw(if (time_us < 10) 10 else time_us);
    TIM14.CR1.modify(.{ .CEN = 1 });
}

pub fn TIM14_IRQHandler() callconv(.C) void {
    TIM14.SR.set_others_one(.{ .UIF = 0 }); // Clear update interrupt flag

    const now = hw.get_ticks();
    if (now - last_scanout_ticks >= 10) {
        last_scanout_ticks = now;
        gpio.configure(board.cols_as_input);
        gpio.configure(board.rows_high_output);
        current_scanout = do_key_scanout();
    }

    const timing = &g_active_timing;

    gpio.configure(board.cols_as_opendrain_highz);
    gpio.configure(board.rows_highz);

    blk: {
        if (timing.num > 0) {
            if (timing.at >= timing.num) {
                timing.at = 0;
                if (timing.off_time != 0) {
                    schedule_next_interrupt(timing.off_time);
                    break :blk;
                }
            }

            const l = timing.matrix_lookup[timing.at];
            rows[l.row].set_mode(.output);
            columns[l.col].set_low();
            schedule_next_interrupt(timing.on_time[timing.at]);
            timing.at += 1;
        } else {
            schedule_next_interrupt(10000);
        }
    }
}
