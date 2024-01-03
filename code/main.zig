const std = @import("std");
const rtt = @import("rtt.zig");
const util = @import("util.zig");
const gpio = @import("gpio.zig");
const hw = @import("hw.zig");
const ir = @import("ir.zig");
const keys = @import("keymatrix.zig");

pub const State = struct {
    last_encoder_count: i16 = 0,
    led_fade_timer: u32 = 0,
    animation: Animation = .idle,
};

const VolumeAnimation = struct {
    t: f32 = 0,
    dir: enum { up, down },
};

const Animation = union(enum) {
    idle,
    volume: VolumeAnimation,
};

fn volume_animation_update(dt: f32, animation: *Animation) void {
    const line = [_]*keys.MatrixNode{
        &keys.matrix[3][0],
        &keys.matrix[0][1],
        &keys.matrix[1][1],
        &keys.matrix[2][1],
    };

    const quarter = animation.volume.t * 3.0;

    for (line, 0..) |l, i| {
        const pos: f32 = @floatFromInt(i);
        const x = @abs(pos - quarter);
        l.brightness = 1.5 - x;
        l.brightness = std.math.clamp(l.brightness, 0, 1);
    }

    switch (animation.volume.dir) {
        .up => animation.volume.t -= dt * 0.003,
        .down => animation.volume.t += dt * 0.003,
    }

    if (animation.volume.t >= 1.0 or animation.volume.t < 0.0) {
        animation.* = .idle;
    }
}

fn encoder_update(state: *State) void {
    const new_encoder = hw.read_encoder();
    if (new_encoder != state.last_encoder_count) {
        const diff = new_encoder - state.last_encoder_count;
        const new_anim = if (diff > 0) VolumeAnimation{
            .t = 1,
            .dir = .up,
        } else VolumeAnimation{
            .t = 0,
            .dir = .down,
        };

        switch (state.animation) {
            .volume => state.animation.volume.dir = new_anim.dir,
            else => state.animation = Animation{ .volume = new_anim },
        }

        state.last_encoder_count = new_encoder;
        rtt.println("{}", .{new_encoder});
    }
}

fn led_animation_update(state: *State) void {
    const now = hw.get_ticks();
    const led_elapsed = now -% state.led_fade_timer;
    if (led_elapsed >= 10) {
        state.led_fade_timer = now;
        var updated: bool = false;
        const led_elapsed_f32: f32 = @floatFromInt(led_elapsed);

        switch (state.animation) {
            .idle => {
                for (0..keys.rows.len) |row_index| {
                    for (0..keys.columns.len) |column_index| {
                        const node = &keys.matrix[row_index][column_index];
                        if (node.input_state) {
                            if (node.brightness != 1) {
                                node.brightness = 1;
                                updated = true;
                            }
                        } else if (node.brightness > 0) {
                            const fade_time_seconds: f32 = 0.5;
                            const step: f32 = (0.001 / fade_time_seconds) * led_elapsed_f32;
                            node.brightness -= step;
                            if (node.brightness < 0) node.brightness = 0;
                            updated = true;
                        }
                    }
                }
            },
            .volume => {
                volume_animation_update(led_elapsed_f32, &state.animation);
                updated = true;
            },
        }

        if (updated) {
            keys.led_setup();
        }
    }
}

var decoder: ir.Decoder = undefined;

pub fn main() noreturn {
    rtt.print_channel_0("Hello\n");

    keys.schedule_next_interrupt(10000);

    //keys.matrix[2][2].brightness = 0.1;
    keys.led_setup();

    ir.enable_receiver(&decoder);

    var state: State = .{};

    while (true) {
        decoder.process();
        encoder_update(&state);
        keys.scanout_update();
        led_animation_update(&state);

        util.wfi();
    }
}
