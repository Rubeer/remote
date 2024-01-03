const rtt = @import("rtt.zig");
const std = @import("std");
const regs = @import("board.zig").regs;

pub const GPIO_Type = @TypeOf(regs.GPIOA);
pub const GPIOA: GPIO_Type = @ptrCast(regs.GPIOA);
pub const GPIOB: GPIO_Type = @ptrCast(regs.GPIOB);
pub const GPIOC: GPIO_Type = @ptrCast(regs.GPIOC);
pub const GPIOD: GPIO_Type = @ptrCast(regs.GPIOD);

const ports = [_]GPIO_Type{
    GPIOA,
    GPIOB,
    GPIOC,
    GPIOD,
};

pub fn get_port(num: u2) GPIO_Type {
    if (false) {
        return ports[num];
    } else {
        const base = @intFromPtr(GPIOA);
        const offset = @intFromPtr(GPIOB) - @intFromPtr(GPIOA);
        if (offset != 0x400) @compileError("Unexpected GPIO offset");
        return @ptrFromInt(base + (@as(u32, num) * offset));
    }
}

pub const Mode = enum(u2) {
    input = 0b00,
    output = 0b01,
    alternate = 0b10,
    analog = 0b11,
};

pub const Otype = enum(u1) {
    push_pull = 0,
    open_drain = 1,
};

pub const Pull = enum(u2) {
    none = 0b00,
    up = 0b01,
    down = 0b10,
    _reserved = 0b11,
};

pub const Level = enum(u1) {
    low = 0,
    high = 1,
};

pub const Altmode = enum(u4) {
    AF0 = 0,
    AF1 = 1,
    AF2 = 2,
    AF3 = 3,
    AF4 = 4,
    AF5 = 5,
    AF6 = 6,
    AF7 = 7,
};

pub const Config = struct {
    name: []const u8,
    mode: ?Mode = null,
    level: ?Level = null,
    pull: ?Pull = null,
    output_type: ?Otype = null,
    alt_mode: ?Altmode = null,
};

pub const Pin = struct {
    port: u2,
    pin: u4,

    pub inline fn set_high(self: Pin) void {
        get_port(self.port).BSRR.write_raw(@as(u32, 1) << self.pin);
    }
    pub inline fn set_mode(self: Pin, mode: Mode) void {
        const shift = 2 * @as(u5, self.pin);
        const mask = ~(@as(u32, 0b11) << shift);
        const bits = @as(u32, @intFromEnum(mode)) << shift;
        var moder = get_port(self.port).MODER.read_raw();
        moder &= mask;
        moder |= bits;
        get_port(self.port).MODER.write_raw(moder);
    }
    pub inline fn set_low(self: Pin) void {
        get_port(self.port).BRR.write_raw((@as(u32, 1) << self.pin));
    }
};

pub fn parse_pin(comptime name: []const u8) Pin {
    const give_example = "Invalid pin name: \"" ++ name ++ "\". " ++ "Use e.g. PA2, PB14";

    if (name[0] != 'P') {
        @compileError(give_example);
    }
    if (name.len > 4) {
        @compileError(give_example);
    }
    if (name[1] < 'A' or name[1] > 'D') {
        @compileError(give_example);
    }

    const pin = comptime std.fmt.parseUnsigned(u4, name[2..], 10) catch |err| @compileError("Cannot parse pin number in " ++ name ++ ": " ++ @errorName(err));

    return Pin{ .port = name[1] - 'A', .pin = pin };
}

fn get_port_num(port_test: GPIO_Type) u2 {
    inline for (ports, 0..) |port, i| {
        if (port == port_test) {
            return i;
        }
    }
    unreachable;
}

const RegMod = struct {
    // Represented as u64 for AFRL/AFRH
    mask: u64,
    bits: u64,

    pub fn upper(self: RegMod) RegMod {
        return RegMod{
            .mask = self.mask >> 32,
            .bits = self.bits >> 32,
        };
    }
};

fn combine_writes(comptime config: anytype, comptime fieldname: []const u8) [ports.len]RegMod {
    var result = std.mem.zeroes([ports.len]RegMod);

    inline for (config) |config_entry| {
        if (@field(config_entry, fieldname)) |field| {
            const field_int = @intFromEnum(field);
            const field_int_type = @TypeOf(field_int);
            const pin = parse_pin(config_entry.name);

            const mask = @as(u64, std.math.maxInt(field_int_type));
            const shift = pin.pin * @typeInfo(field_int_type).Int.bits;
            result[pin.port].mask |= mask << shift;
            result[pin.port].bits |= @as(u64, field_int) << shift;
        }
    }

    return result;
}

fn apply_regmod(addr: *volatile u32, comptime mod: RegMod) void {
    if (mod.mask != 0) {
        var tmp = addr.*;
        tmp &= @truncate(~mod.mask);
        tmp |= @truncate(mod.bits);
        addr.* = tmp;
    }
}

pub fn configure(comptime config: anytype) void {
    const level = comptime combine_writes(config, "level");
    const output_type = comptime combine_writes(config, "output_type");
    const mode = comptime combine_writes(config, "mode");
    const pull = comptime combine_writes(config, "pull");
    const altmode = comptime combine_writes(config, "alt_mode");

    inline for (ports, 0..) |port, i| {
        if (level[i].mask != 0) {
            const set_bits = level[i].bits;
            const reset_bits = level[i].bits ^ level[i].mask;
            const combined = set_bits | (reset_bits << 16);

            if (combined != 0) {
                port.BSRR.write_raw(combined);
            }
        }

        apply_regmod(&port.OTYPER.raw, output_type[i]);
        apply_regmod(&port.MODER.raw, mode[i]);
        apply_regmod(&port.PUPDR.raw, pull[i]);

        apply_regmod(&port.AFRL.raw, altmode[i]);
        apply_regmod(&port.AFRH.raw, altmode[i].upper());
    }
}
