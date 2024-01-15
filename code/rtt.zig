// The contents of this file is dual-licensed under the MIT or 0BSD license.
// NOTE(robin): Modified from: https://github.com/rjsberry/rtt.zig

//! A target-side implementation of the Real-Time Transfer protocol.
//!
//! The library provides a single writeable _up-channel_ for sending log data to
//! the host. The functions in this library are also not reentrant; concurrent
//! use must be protected with a critical section.
//!
//! Functions will block if the RTT buffer is full. If the host-side has
//! disconnected and is no longer reading from the channel the microcontroller
//! will lock up.
//!
//! # Examples
//!
//! ```zig
//! const rtt = @import("rtt");
//!
//! export fn main(void) callconv(.C) noreturn {
//!     rtt.println("Hello from {s}", .{"Zig!"});
//!     while (true) {}
//! }
//! ```

const std = @import("std");
const util = @import("util.zig");

const atomic = std.atomic;
const debug = std.debug;
const fmt = std.fmt;

/// Print something to the RTT channel.
///
/// Uses the `std.fmt` plumbing under the hood.
pub fn print(comptime fmt_str: []const u8, args: anytype) void {
    fmt.format(Writer{}, fmt_str, args) catch unreachable;
}

/// Print something to the RTT channel, with a newline.
///
/// Uses the `std.fmt` plumbing under the hood.
pub fn println(comptime fmt_str: []const u8, args: anytype) void {
    fmt.format(Writer{}, fmt_str ++ "\n", args) catch unreachable;
}

// NOTE(robin): noinline and hardcoded to reduce code bloat
/// Print without formatting to up channel 0
pub noinline fn print_channel_0(bytes: []const u8) void {
    const chan = &_SEGGER_RTT.channel;
    const buf = &BUF;

    var xs = bytes;

    var write = readVolatile(&chan.write);
    const read = readVolatile(&chan.read);

    inline for (0..2) |i| {
        const avail = maxContiguous(read, write);
        const n = @min(xs.len, avail);
        if (i == 1 and n == 0) break;
        @memcpy(buf[write .. write + n], xs[0..n]);
        xs = xs[n..];
        write = (write + n) & (buf.len - 1);
    }

    util.full_memory_barrier();
    writeVolatile(&chan.write, write);
}

const RTT_MODE_TRUNC = 1;
const RTT_MODE_BLOCK = 2;

var BUF: [1024]u8 align(4) linksection(".rtt") = undefined;

export var _SEGGER_RTT: extern struct {
    magic: [16]u8,
    max_up_channels: u32,
    max_down_channels: u32,
    channel: Channel,
} align(4) linksection(".rtt") = undefined;

const Channel = extern struct {
    name: [*:0]const u8,
    buf: *[BUF.len]u8,
    bufsz: u32,
    write: u32,
    read: u32,
    flags: u32,
};

pub fn init() void {
    _SEGGER_RTT.max_up_channels = 1;
    _SEGGER_RTT.max_down_channels = 0;
    _SEGGER_RTT.channel = .{
        .name = "rtt.zig",
        .buf = &BUF,
        .bufsz = BUF.len,
        .write = 0,
        .read = 0,
        .flags = RTT_MODE_TRUNC,
    };

    // Copy in separate parts so we don't accidently end up with
    // the string "SEGGER RTT" somewhere else in memory
    // (Which may cause the RTT control block to be detected in the wrong place)
    util.full_memory_barrier();
    _SEGGER_RTT.magic[7..10].* = "RTT".*;
    util.full_memory_barrier();
    _SEGGER_RTT.magic[0..6].* = "SEGGER".*;
    util.full_memory_barrier();
    _SEGGER_RTT.magic[6] = ' ';
    util.full_memory_barrier();
}

const Writer = struct {
    pub const Error = error{}; // infallible

    pub inline fn writeAll(self: Writer, bytes: []const u8) Writer.Error!void {
        _ = self;
        print_channel_0(bytes);
    }

    pub inline fn writeBytesNTimes(self: Writer, bytes: []const u8, n: usize) Writer.Error!void {
        for (0..n) |_| try self.writeAll(bytes);
    }
};

inline fn readVolatile(ptr: *const volatile u32) u32 {
    return ptr.*;
}

inline fn writeVolatile(ptr: *volatile u32, val: u32) void {
    ptr.* = val;
}

inline fn maxContiguous(read: u32, write: u32) u32 {
    return if (read > write)
        read - write - 1
    else if (read == 0)
        BUF.len - write - 1
    else
        BUF.len - write;
}
