const std = @import("std");
const assert = std.debug.assert;

pub fn Mmio(comptime PackedT: type) type {
    const size = @bitSizeOf(PackedT);
    if ((size % 8) != 0)
        @compileError("size must be divisible by 8!");

    if (!std.math.isPowerOfTwo(size / 8))
        @compileError("size must encode a power of two number of bytes!");

    const IntT = std.meta.Int(.unsigned, size);

    if (@sizeOf(PackedT) != (size / 8))
        @compileError(std.fmt.comptimePrint("IntT and PackedT must have the same size!, they are {} and {} bytes respectively", .{ size / 8, @sizeOf(PackedT) }));

    return extern struct {
        const Self = @This();

        raw: IntT,

        pub const underlying_type = PackedT;

        pub inline fn read(addr: *volatile Self) PackedT {
            return @bitCast(addr.raw);
        }
        pub inline fn read_raw(addr: *volatile Self) IntT {
            return addr.raw;
        }

        pub inline fn write(addr: *volatile Self, val: PackedT) void {
            comptime {
                assert(@bitSizeOf(PackedT) == @bitSizeOf(IntT));
            }
            addr.write_raw(@bitCast(val));
        }

        pub inline fn write_raw(addr: *volatile Self, val: IntT) void {
            addr.raw = val;
        }

        pub inline fn modify(addr: *volatile Self, fields: anytype) void {
            var val = read(addr);
            inline for (@typeInfo(@TypeOf(fields)).Struct.fields) |field| {
                @field(val, field.name) = @field(fields, field.name);
            }
            write(addr, val);
        }

        pub inline fn toggle(addr: *volatile Self, fields: anytype) void {
            var val = read(addr);
            inline for (@typeInfo(@TypeOf(fields)).Struct.fields) |field| {
                @field(val, @tagName(field.default_value.?)) = !@field(val, @tagName(field.default_value.?));
            }
            write(addr, val);
        }

        /// For write-only register where writing 0 does not do anything (e.g. write 1 to clear the flag, write 0 to leave it be)
        pub inline fn set_others_zero(addr: *volatile Self, fields: anytype) void {
            var reg = std.mem.zeroes(PackedT);
            inline for (@typeInfo(@TypeOf(fields)).Struct.fields) |field| {
                @field(reg, field.name) = @field(fields, field.name);
            }
            write(addr, reg);
        }

        /// For write-only register where writing 1 does not do anything (e.g. write 0 to clear the flag, write 1 to leave it be)
        pub inline fn set_others_one(addr: *volatile Self, fields: anytype) void {
            var reg: PackedT = @bitCast(@as(IntT, std.math.maxInt(IntT)));
            inline for (@typeInfo(@TypeOf(fields)).Struct.fields) |field| {
                @field(reg, field.name) = @field(fields, field.name);
            }
            write(addr, reg);
        }
    };
}
