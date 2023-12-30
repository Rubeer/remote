const std = @import("std");
const gpio = @import("gpio.zig");
const board = @import("board.zig");
const util = @import("util.zig");
const rtt = @import("rtt.zig");

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

var previous_row_scan = std.mem.zeroes([rows.len]u8);

fn make_cols_mask() u8 {
    var mask: u32 = 0;
    inline for (columns) |col| {
        mask |= @as(u32, 1) << col.pin;
    }
    return @truncate(mask);
}

fn handle_key_change(row: u8, col: u8, pressed: bool) void {
    rtt.println("Change: {},{} = {}", .{ row, col, pressed });
}

pub fn scanout() void {
    gpio.configure(board.cols_as_input);
    gpio.configure(board.rows_high);

    var row_scan: [rows.len]u8 = undefined;
    for (rows, &row_scan) |row, *bits| {
        row.set_low();

        // Adequate delay with 16MHz clock
        util.nops(15);

        bits.* = @truncate(gpio.GPIOA.IDR.read_raw());

        row.set_high();
    }

    gpio.configure(board.rows_low);

    const cols_mask = comptime make_cols_mask();

    for (row_scan, 0..) |bits, i| {
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
