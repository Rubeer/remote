const std = @import("std");
const gpio = @import("gpio.zig");
const board = @import("board.zig");
const util = @import("util.zig");
const rtt = @import("rtt.zig");

const TIM14 = board.regs.TIM14;
const GPIOA = gpio.GPIOA;

const rows = [_]gpio.Pin{
    gpio.parse_pin(board.pins.row1),
    gpio.parse_pin(board.pins.row2),
    gpio.parse_pin(board.pins.row3),
    gpio.parse_pin(board.pins.row4),
};

const columns = [_]gpio.Pin{
    gpio.parse_pin(board.pins.col1),
    gpio.parse_pin(board.pins.col2),
    gpio.parse_pin(board.pins.col3),
};

pub var matrix: [rows.len][columns.len]MatrixNode = undefined;
var previous_row_scan: [rows.len]u8 = undefined;
var leds: [10]EncodedNode = undefined;
var leds_enabled: u8 = 0;
var led_at: u8 = 0;
var led_off_time: u16 = 0;

const MatrixNode = struct {
    brightness: f32 = 0,
    input_timestamp: u32 = 0,
    led_on_time: u16 = 0,
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
    var num_leds: u8 = 0;
    var total_brightness: f32 = 0;

    for (0..rows.len) |row_index| {
        for (0..columns.len) |column_index| {
            if (row_index == 3 and (column_index == 1 or column_index == 2)) {
                continue;
            }

            const node = &matrix[row_index][column_index];

            if (node.brightness > 0) {
                total_brightness += node.brightness;
                leds[num_leds] = EncodedNode.encode(row_index, column_index);
                num_leds += 1;
            }
        }
    }

    var led_factor: f32 = 1;
    if (total_brightness > 1) {
        led_factor = 1.0 / total_brightness;
    }

    const total_budget = 65535;

    const budget_per_led: f32 = 0.1 * led_factor * @as(f32, @floatFromInt(total_budget));
    var budget_used: u32 = 0;

    for (0..num_leds) |i| {
        const node = leds[i].decode();
        const on_time = node.brightness * budget_per_led;
        node.led_on_time = @intFromFloat(on_time);
        budget_used += node.led_on_time;
    }

    if (total_budget > budget_used) {
        led_off_time = @truncate(total_budget - budget_used);
    } else {
        led_off_time = 0;
    }

    asm volatile ("" ::: "memory");
    leds_enabled = num_leds;
}

fn make_cols_mask() u8 {
    var mask: u32 = 0;
    inline for (columns) |col| {
        mask |= @as(u32, 1) << col.pin;
    }
    return @truncate(mask);
}

fn handle_key_change(row: u8, col: u8, pressed: bool) void {
    matrix[row][col].input_state = pressed;
    matrix[row][col].brightness = if (pressed) 1.0 else 0.0;
    led_setup();
    rtt.println("Change: {},{} = {}", .{ row, col, pressed });
}

var time_since_last_scanout: u16 = 0;

fn do_key_scanout() [rows.len]u8 {
    var result: [rows.len]u8 = undefined;
    time_since_last_scanout = 0;
    gpio.configure(board.cols_as_input);
    gpio.configure(board.rows_high);

    for (rows, &result) |row, *result_row| {
        row.set_low();

        // Adequate delay with 16MHz clock
        util.nops(15);

        result_row.* = @truncate(GPIOA.IDR.read_raw());

        row.set_high();
    }

    gpio.configure(board.rows_low);
    return result;
}

fn process_scanout(scanout: [rows.len]u8) void {
    const cols_mask = comptime make_cols_mask();

    for (scanout, 0..) |bits, i| {
        const diff = bits ^ previous_row_scan[i];
        if (diff & cols_mask != 0) {
            for (columns, 0..) |col, j| {
                const bit = @as(u32, 1) << col.pin;
                if (diff & bit != 0) {
                    handle_key_change(@truncate(i), @truncate(j), bits & bit == 0);
                }
            }

            previous_row_scan[i] = bits;
        }
    }
}

pub fn schedule_next_interrupt(time_us: u16) void {
    TIM14.ARR.write_raw(time_us);
    TIM14.CR1.modify(.{ .CEN = 1 });
}

pub fn TIM14_IRQHandler() callconv(.C) void {
    TIM14.SR.set_others_one(.{ .UIF = 0 }); // Clear update interrupt flag

    //time_since_last_scanout += TIM14.CNT.read().CNT;
    var key_scanout: ?[rows.len]u8 = null;
    if (true) { //time_since_last_scanout > 5000) {
        time_since_last_scanout = 0;
        key_scanout = do_key_scanout();
    }

    if (leds_enabled != 0) {
        gpio.configure(board.rows_low);
        gpio.configure(board.cols_as_opendrain_highz);

        blk: {
            if (led_at >= leds_enabled) {
                led_at = 0;
                if (led_off_time != 0) {
                    schedule_next_interrupt(led_off_time);
                    break :blk;
                }
            }
            const led = leds[led_at];
            rows[led.row].set_high();
            columns[led.col].set_low();
            schedule_next_interrupt(led.decode().led_on_time);
        }
    } else {
        schedule_next_interrupt(10000); // 10 ms
    }

    if (key_scanout) |scanout| {
        process_scanout(scanout);
    }
}
