const std = @import("std");
const hw = @import("hw.zig");
const regs = @import("STM32G030.zig").devices.STM32G030.peripherals;
const gpio = @import("gpio.zig");
const board = @import("board.zig");
const rtt = @import("rtt.zig");
const util = @import("util.zig");
const builtin = @import("builtin");

// Receiver
const TIM1 = regs.TIM1;

// Transmitter
const TIM16 = regs.TIM16;
// Modulation
const TIM17 = regs.TIM17;

const DMA = regs.DMA;
const RCC = regs.RCC;

pub const IRCommand = struct {
    carrier_frequency: u16,
    timings: []const u16,
};

fn force_constant(comptime tuple: anytype) []const u16 {
    const inner = struct {
        const array: [tuple.len]u16 = tuple;
    };
    return &inner.array;
}

pub const panasonic_tv_power_toggle = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 54887, 28303, 6536, 7394, 6533, 21331, 6531, 7394, 6535, 7394, 6531, 7393, 6535, 7394, 6532, 7393, 6536, 7393, 6536, 7394, 6536, 7395, 6536, 7394, 6537, 7395, 6536, 7394, 6536, 21335, 6535, 7393, 6532, 7394, 6532, 7393, 6537, 7394, 6531, 7394, 6537, 7393, 6536, 7395, 6532, 7394, 6531, 7394, 6536, 21332, 6535, 7392, 6535, 7394, 6533, 7394, 6531, 7394, 6532, 7392, 6535, 7393, 6535, 7394, 6532, 7393, 6536, 21330, 6531, 7394, 6532, 21332, 6532, 21334, 6536, 21333, 6536, 21335, 6536, 7394, 6537, 7394, 6536, 21333, 6530, 7392, 6536, 21336, 6536, 21333, 6532, 21334, 6531, 21333, 6536, 7394, 6537, 21334, 6535, 0 }),
};

pub const panasonic_tv_on = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 55073, 28206, 6611, 7489, 6612, 21588, 6175, 7930, 6168, 7492, 6617, 7489, 6616, 7491, 6614, 7489, 6169, 7933, 6172, 7934, 6171, 7934, 6173, 7491, 6611, 7489, 6612, 7491, 6610, 21596, 6159, 7930, 6168, 7490, 6610, 7489, 6605, 7489, 6609, 7490, 6172, 7931, 6173, 7931, 6171, 7932, 6173, 7491, 6617, 21593, 6167, 7930, 6168, 7934, 6170, 7924, 6159, 7492, 6610, 7492, 6612, 7489, 6612, 7489, 6171, 7932, 6171, 7933, 6175, 21599, 6612, 21603, 6170, 21598, 6614, 21586, 6165, 21595, 6602, 7487, 6609, 7489, 6164, 7921, 6173, 21598, 6611, 21598, 6174, 21596, 6610, 21601, 6175, 21597, 6614, 7489, 6606, 21597, 6166, 0 }),
};

pub const panasonic_tv_off = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 55072, 28185, 6610, 7474, 6170, 21578, 6610, 7477, 6606, 7476, 6610, 7479, 6170, 7917, 6167, 7917, 6167, 7917, 6166, 7475, 6611, 7476, 6607, 7476, 6610, 7476, 6611, 7476, 6171, 21576, 6605, 7474, 6605, 7474, 6606, 7475, 6172, 7916, 6169, 7915, 6166, 7914, 6169, 7474, 6607, 7473, 6603, 7474, 6606, 21572, 6169, 7916, 6170, 7478, 6607, 7477, 6612, 7474, 6612, 7475, 6170, 7918, 6166, 7919, 6171, 7918, 6171, 21579, 6612, 21583, 6167, 21582, 6612, 21581, 6170, 21579, 6612, 21580, 6170, 7476, 6611, 7476, 6611, 21580, 6170, 21577, 6611, 21580, 6170, 21579, 6610, 21581, 6166, 21576, 6608, 7475, 6605, 21580, 6168, 0 }),
};

pub const panasonic_volume_up = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 53595, 28339, 5230, 7421, 5230, 21371, 5231, 7420, 5231, 7421, 5231, 7420, 5232, 7420, 5230, 7419, 5231, 7419, 5230, 7418, 5230, 7420, 5231, 7420, 5229, 7418, 5230, 7420, 5231, 21373, 5232, 7421, 5231, 7421, 5230, 7420, 5229, 7419, 5232, 7420, 5230, 7419, 5229, 7419, 5230, 7420, 5231, 7418, 5229, 21367, 5230, 7417, 5230, 7419, 5231, 7420, 5229, 7418, 5230, 7418, 5229, 7417, 5230, 7420, 5229, 7418, 5230, 7417, 5231, 7419, 5232, 7417, 5231, 7419, 5228, 7420, 5231, 21368, 5230, 7418, 5229, 7418, 5229, 7418, 5230, 7418, 5232, 7420, 5231, 7420, 5231, 7417, 5230, 21368, 5231, 7420, 5232, 21371, 5232, 0 }),
};

pub const panasonic_volume_down = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 53579, 28308, 5229, 7396, 5229, 21341, 5228, 7398, 5229, 7397, 5229, 7397, 5230, 7397, 5229, 7396, 5229, 7396, 5230, 7398, 5230, 7397, 5229, 7397, 5228, 7396, 5228, 7397, 5228, 21343, 5230, 7398, 5228, 7398, 5230, 7399, 5229, 7397, 5229, 7397, 5229, 7398, 5229, 7399, 5229, 7398, 5231, 7399, 5230, 21345, 5230, 7400, 5230, 7397, 5229, 7397, 5228, 7397, 5229, 7397, 5228, 7397, 5228, 7396, 5229, 7397, 5229, 21340, 5228, 7396, 5228, 7397, 5228, 7397, 5229, 7396, 5228, 21339, 5227, 7395, 5228, 7395, 5228, 21335, 5227, 7395, 5227, 7395, 5229, 7394, 5228, 7398, 5229, 21338, 5228, 7396, 5227, 21336, 5228, 0 }),
};

pub const panasonic_hdmi1 = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 55071, 28170, 6609, 7467, 6165, 21567, 6611, 7462, 6609, 7465, 6606, 7463, 6169, 7906, 6173, 7904, 6169, 7903, 6174, 7905, 6165, 7464, 6603, 7462, 6606, 7463, 6606, 7463, 6169, 21562, 6605, 7464, 6606, 7462, 6607, 7466, 6171, 7901, 6166, 7906, 6170, 7905, 6168, 7904, 6167, 7463, 6603, 7465, 6608, 21563, 6164, 7903, 6167, 7465, 6606, 21563, 6168, 7906, 6170, 7905, 6169, 7907, 6170, 7464, 6607, 7466, 6606, 7464, 6611, 7465, 6173, 7907, 6169, 7905, 6166, 21564, 6604, 21559, 6170, 7903, 6164, 21553, 6602, 7464, 6173, 7905, 6172, 21563, 6610, 7464, 6614, 21565, 6170, 21563, 6614, 7466, 6174, 7906, 6170, 0 }),
};

pub const panasonic_hdmi2 = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 55515, 28189, 6173, 7921, 6164, 21577, 6614, 7482, 6612, 7482, 6173, 7923, 6166, 7924, 6163, 7921, 6174, 7482, 6616, 7484, 6615, 7483, 6609, 7481, 6614, 7483, 6174, 7923, 6168, 21582, 6614, 7481, 6614, 7481, 6171, 7925, 6168, 7927, 6167, 7922, 6172, 7482, 6609, 7482, 6609, 7481, 6614, 7483, 6609, 21584, 6163, 7482, 6615, 7481, 6614, 21587, 6171, 7924, 6167, 7923, 6173, 7482, 6608, 7483, 6608, 7483, 6609, 21584, 6163, 7482, 6614, 7482, 6607, 7481, 6614, 21582, 6171, 21581, 6614, 7482, 6608, 21580, 6166, 21581, 6615, 7483, 6169, 21580, 6608, 7481, 6604, 21578, 6175, 21579, 6612, 7481, 6608, 7482, 6175, 0 }),
};

pub const power_switch_main_on = IRCommand{
    .carrier_frequency = 36000,
    //.timings = force_constant(.{ 40834, 14762, 5325, 14765, 5327, 7568, 5327, 7568, 5326, 14763, 12425, 7504, 12428, 14700, 5327, 7566, 5326, 7569, 12427, 7502, 5325, 14764, 5326, 7568, 5325, 7565, 5325, 7568, 12427, 7504, 5328, 7567, 5328, 14768, 12429, 0 }),
    .timings = force_constant(.{ 42162, 14758, 6658, 14759, 6659, 7562, 6658, 7562, 20860, 21829, 13759, 14697, 6659, 7563, 6660, 7562, 13757, 7499, 6660, 14760, 6660, 7562, 6659, 7562, 6658, 7562, 13762, 7499, 6659, 7563, 6659, 14762, 13762, 0 }),
};

pub const power_switch_main_off = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 42174, 14760, 6660, 14762, 6659, 7566, 6661, 7565, 6659, 14762, 13760, 7500, 13760, 14698, 6659, 7564, 6659, 7564, 13761, 7500, 6663, 14764, 6659, 7566, 6661, 7564, 6660, 7564, 6659, 7564, 13764, 14700, 6660, 7565, 13764, 0 }),
};

pub const power_switch_secondary_on = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 42167, 14755, 6658, 14754, 6659, 7558, 6658, 7558, 6658, 14756, 13761, 7496, 13761, 14691, 6658, 7558, 6659, 7560, 13760, 7496, 6660, 14758, 6659, 7560, 6660, 7560, 6659, 7559, 6658, 7559, 6659, 7559, 6659, 7558, 13761, 7494, 6659, 0 }),
};

pub const power_switch_secondary_off = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 42179, 14756, 6660, 14759, 6660, 7559, 6659, 7560, 20865, 21825, 13762, 14691, 6658, 7559, 6661, 7560, 13763, 7496, 6659, 14758, 6659, 7559, 6660, 7559, 6655, 7560, 6659, 7559, 6660, 7560, 13763, 7496, 6660, 14758, 6660, 0 }),
};

pub const nad_amp_on = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 12672, 7168, 8455, 27470, 8880, 27469, 8457, 27471, 8458, 9713, 8457, 9291, 8881, 9290, 8452, 9713, 8457, 27471, 8458, 9713, 8457, 9289, 8880, 27471, 8458, 27466, 8457, 27470, 8881, 27472, 8458, 27473, 8457, 9714, 8458, 27473, 8458, 9712, 8453, 27473, 8458, 9713, 8457, 9289, 8879, 27470, 8455, 9289, 8874, 9288, 8455, 9712, 8456, 27466, 8458, 9712, 8455, 27466, 8452, 27464, 8877, 9289, 8455, 27465, 8879, 27459, 8457, 0 }),
};

pub const nad_amp_off = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 12626, 7140, 8449, 27461, 8450, 27459, 8878, 27464, 8455, 9289, 8878, 9289, 8450, 9713, 8451, 9291, 8873, 27463, 8455, 9293, 8878, 9291, 8455, 27460, 8876, 27457, 8451, 27458, 8448, 27459, 8876, 27455, 8453, 9289, 8875, 9290, 8450, 9712, 8449, 9288, 8873, 27464, 8456, 9290, 8877, 9288, 8454, 27460, 8872, 27461, 8452, 27459, 8454, 27459, 8877, 27462, 8454, 9289, 8878, 27461, 8454, 27461, 8450, 9711, 8449, 9287, 8876, 0 }),
};

pub const nad_amp_input_aux = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 12650, 7170, 8456, 27475, 8457, 27474, 8880, 27455, 8451, 9298, 8880, 9297, 8457, 9720, 8457, 9296, 8874, 27473, 8457, 9294, 8880, 9296, 8457, 27470, 8875, 27469, 8456, 27471, 8456, 27471, 8875, 27468, 8456, 9296, 8878, 27469, 8456, 27476, 8458, 9721, 8456, 27475, 8457, 27473, 8875, 9295, 8455, 9718, 8456, 27470, 8456, 9718, 8455, 9295, 8878, 27472, 8456, 9296, 8874, 9294, 8452, 27468, 8875, 27467, 8453, 9293, 8872, 0 }),
};

pub const nad_amp_input_video2 = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 12690, 7178, 8456, 27479, 8880, 27480, 8456, 27470, 8458, 9721, 8453, 9297, 8880, 9297, 8456, 9719, 8456, 27475, 8452, 9722, 8458, 9298, 8876, 27476, 8458, 27476, 8457, 27473, 8876, 27474, 8456, 27474, 8459, 9722, 8458, 9297, 8879, 9297, 8457, 9720, 8457, 9298, 8879, 9298, 8458, 9720, 8452, 27471, 8456, 27472, 8879, 27478, 8456, 27475, 8457, 27477, 8881, 27478, 8457, 27477, 8456, 27482, 8882, 9300, 8457, 9722, 8457, 0 }),
};

pub const nad_amp_volume_up = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 12781, 6798, 8417, 27328, 8415, 27329, 8416, 27330, 8417, 9242, 8836, 9241, 8832, 9241, 8412, 9240, 8823, 26892, 8834, 9243, 8838, 9242, 8833, 26911, 8833, 26913, 8836, 26912, 8837, 26912, 8839, 26911, 8836, 9242, 8417, 9243, 8836, 9241, 8837, 9241, 8412, 27331, 8412, 9243, 8839, 9244, 8837, 9241, 8836, 26912, 8836, 26908, 8835, 26914, 8831, 26905, 8837, 9242, 8416, 27330, 8416, 27336, 8417, 27323, 8413, 9242, 8839, 0 }),
};

pub const nad_amp_volume_down = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 12830, 6799, 8416, 27326, 8415, 27331, 8419, 27331, 8418, 9235, 8835, 9236, 8833, 9232, 8834, 9235, 8414, 27324, 8414, 9235, 8835, 9238, 8833, 26909, 8836, 26906, 8839, 26903, 8835, 26903, 8832, 26904, 8837, 9235, 8414, 9236, 8835, 9236, 8832, 26904, 8833, 26902, 8832, 9235, 8832, 9236, 8415, 9236, 8836, 26905, 8838, 26906, 8837, 26910, 8837, 9236, 8837, 9236, 8415, 27314, 8414, 27323, 8414, 27328, 8410, 9236, 8833, 0 }),
};

pub const nad_amp_mute_toggle = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 12805, 6808, 8832, 26906, 8835, 26903, 8835, 26903, 8831, 9235, 8415, 9235, 8835, 9238, 8834, 9237, 8416, 27323, 8414, 9238, 8835, 9236, 8836, 26899, 8830, 26898, 8831, 26891, 8836, 26906, 8833, 26904, 8831, 9235, 8837, 9236, 8415, 9236, 8834, 26905, 8833, 9237, 8834, 26905, 8836, 9235, 8415, 9235, 8831, 26904, 8832, 26904, 8837, 26913, 8837, 9239, 8837, 26911, 8838, 9235, 8833, 26909, 8837, 26911, 8830, 9236, 8415, 0 }),
};

pub const fan_0 = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 19865, 7161, 19864, 7161, 6341, 20685, 6338, 20688, 6342, 20687, 6342, 20684, 6342, 20685, 19866, 7159, 6339, 20680, 6341, 20683, 6341, 20680, 6336, 0 }),
};
pub const fan_1 = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 19865, 7160, 19863, 7162, 6340, 20686, 6341, 20686, 6336, 20688, 6342, 20686, 6341, 20688, 6340, 20687, 6341, 20686, 6342, 20685, 6337, 20685, 19860, 0 }),
};

pub const fan_2 = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 19861, 7159, 19862, 7160, 6337, 20679, 6336, 20683, 6340, 20676, 6336, 20680, 6337, 20681, 6337, 20680, 6339, 20679, 19860, 7159, 6339, 20674, 6339, 0 }),
};
pub const fan_3 = IRCommand{
    .carrier_frequency = 38000,
    .timings = force_constant(.{ 19859, 7161, 19860, 7162, 6341, 20683, 6337, 20683, 6342, 20686, 19868, 7163, 6341, 20688, 6338, 20684, 6340, 20684, 6341, 20683, 19859, 7160, 19856, 0 }),
};

const Sequence = struct {
    const Type = enum {
        ir_command,
        delay,
    };

    const Entry = union(Type) {
        ir_command: *const IRCommand,
        delay: u32, // milliseconds
    };

    pub fn add_entry(self: *@This(), entry: Entry) !void {
        if (!builtin.is_test) {
            util.disable_interrupts();
            defer util.enable_interrupts();
            if (self.entries.writeItem(entry)) |_| {
                if (!util.read_volatile(&sequence.running)) {
                    self.run_entry();
                }
            } else |_| {}
        } else {
            try self.entries.writeItem(entry);
        }
    }

    pub fn run_entry(self: *@This()) void {
        if (self.running) {
            rtt.println("Already running!", .{});
            return;
        }

        if (self.entries.readItem()) |entry| {
            util.write_volatile(&sequence.running, true);
            switch (entry) {
                .ir_command => |ir_command| {
                    transmit(ir_command);
                },
                .delay => |delay| {
                    start_delay(delay);
                },
            }
        } else {
            rtt.println("End of sequence", .{});
        }
    }

    pub fn run_entry_test(self: *@This()) error{NoWork}!void {
        if (self.entries.readItem()) |entry| {
            switch (entry) {
                .ir_command => |ir_command| {
                    std.debug.print("freq: {}, num: {}\n", .{ ir_command.carrier_frequency, ir_command.timings.len });
                },
                .delay => |delay| {
                    std.debug.print("delay: {}\n", .{delay});
                },
            }
        } else {
            return error.NoWork;
        }
    }

    pub fn init(self: *@This()) void {
        self.entries = @TypeOf(self.entries).init();
    }

    entries: std.fifo.LinearFifo(Entry, .{ .Static = 64 }),

    running: bool,
};

test "sequence" {
    var s: Sequence = undefined;
    s.init();
    try s.add_entry(.{ .delay = 123 });
    try s.add_entry(.{ .delay = 456 });
    try s.add_entry(.{ .ir_command = &fan_3 });

    while (true) {
        s.run_entry_test() catch break;
    }

    try s.add_entry(.{ .delay = 11 });
    try s.add_entry(.{ .delay = 11 });
    try s.add_entry(.{ .delay = 11 });
    try s.add_entry(.{ .ir_command = &nad_amp_input_aux });
    try s.add_entry(.{ .delay = 11 });
    try s.add_entry(.{ .delay = 456 });
    try s.add_entry(.{ .ir_command = &fan_3 });

    while (true) {
        s.run_entry_test() catch break;
    }
}

pub var sequence: Sequence = undefined;

pub fn start_delay(delay: u32) void {
    // Setup TIM16 so it triggers TIM16_IRQHandler after the delay.
    // TIM16->CNT = 0;

    // assert(SystemCoreClock == 16000000);
    // TIM16->PSC = 15999;

    // TIM16->ARR = entry->delay_ms;
    // LL_TIM_EnableIT_UPDATE(TIM16);
    // LL_TIM_EnableCounter(TIM16);

    RCC.APBENR2.modify(.{
        .TIM16EN = 1,
    });

    rtt.println("delay {}", .{delay});
    TIM16.CNT.write_raw(0);
    TIM16.PSC.write_raw(15999);
    TIM16.ARR.write_raw(delay);
    //if (TIM16.SR.read().UIF == 1) {
    //TIM16.SR.set_others_one(.{ .UIF = 0 });
    //}
    //TIM16.SR.set_others_one(.{ .UIF = 0 });
    TIM16.DIER.modify(.{ .UIE = 1 });
    TIM16.CR1.modify(.{ .CEN = 1 });
}

pub fn TIM16_IRQHandler() callconv(.C) void {
    if (TIM16.SR.read().UIF == 1) {
        TIM16.CR1.modify(.{ .CEN = 0 });
        TIM16.DIER.modify(.{ .UIE = 0 });
        TIM16.SR.set_others_one(.{ .UIF = 0 });
        rtt.println("tim16", .{});
        RCC.APBENR2.modify(.{
            .TIM16EN = 0,
        });
        util.write_volatile(&sequence.running, false);
        sequence.run_entry();
    } else {
        rtt.println("tim16 spurious", .{});
        TIM16.SR.write_raw(0); //(.{ .UIF = 0 });
    }
}

pub fn transmit(cmd: *const IRCommand) void {
    // TIM16 outputs the waveform without the carrier frequency.
    // TIM17 generates the carrier frequency modulation (AND-ed with TIM16 output)
    // (This was setup in hw.init, SYSCFG.CFGR1 register)

    const freq: u32 = cmd.carrier_frequency;
    const carrier_period = (hw.clock_rate + freq / 2) / freq;

    rtt.println("Transmit begin, {} pulses. Carrier: {} Hz ({})", .{
        cmd.timings.len,
        cmd.carrier_frequency,
        carrier_period,
    });

    RCC.APBENR2.modify(.{
        .TIM16EN = 1,
        .TIM17EN = 1,
    });

    DMA.CCR1.modify(.{ .EN = 0 });
    DMA.CNDTR1.write_raw(cmd.timings.len);
    DMA.CMAR1.write_raw(@intFromPtr(cmd.timings.ptr));

    TIM17.CCR1.write_raw(carrier_period / 3); // 1/3 carrier duty cycle
    TIM17.ARR.write_raw(carrier_period - 1);
    TIM17.CNT.write_raw(0);
    TIM17.CCER.modify(.{ .CC1E = 1 }); // Enable CC channel 1

    TIM16.PSC.write_raw(0);
    TIM16.ARR.write_raw(cmd.timings[0]);
    TIM16.CNT.write_raw(0);
    TIM16.CCER.modify(.{ .CC1E = 1 }); // Enable CC channel 1

    // Force the output level to begin at inactive level
    // (It may be in another state due to previous toggles)
    var reg = TIM16.CCMR1_Output.read();
    reg.OC1M = .forced_inactive;
    TIM16.CCMR1_Output.write(reg);
    reg.OC1M = .toggle;
    TIM16.CCMR1_Output.write(reg);

    DMA.CCR1.modify(.{ .EN = 1 });
    TIM16.DIER.modify(.{ .CC1DE = 1 }); // Enable DMA request on CC channel 1

    // Switches the output pin from output mode to alternate mode.
    // (i.e. connects the pin to the timer output)
    gpio.configure(board.ir_output_on);

    // Enable counters
    TIM16.CR1.modify(.{ .CEN = 1 });
    TIM17.CR1.modify(.{ .CEN = 1 });
}

comptime {
    const decoder: Decoder = undefined;
    if (!std.math.isPowerOfTwo(decoder.raw_timing.len)) {
        @compileError("Must be a power of two");
    }
}

pub const Decoder = struct {
    raw_timing: [128]u16,
    prev_sample: u16,

    at: u32,
    in_carrier_count: u32,
    carrier_guesses: [4]u16,

    pulse_start: u16,

    pulses: std.BoundedArray(packed struct { high: u16, low: u16 }, 128),

    pub fn process(self: *Decoder) void {
        const dma_counter = DMA.CNDTR2.read_raw();

        const dma_at = if (dma_counter != 0)
            self.raw_timing.len - dma_counter
        else
            0;

        while (dma_at != self.at) {
            const sample = util.read_volatile(&self.raw_timing[self.at]);
            const diff = sample -% self.prev_sample;
            //rtt.println("{}", .{diff});

            const period_20KHz = hw.clock_rate / 20000;
            const period_30KHz = hw.clock_rate / 30000;
            const period_45KHz = hw.clock_rate / 45000;

            if (self.in_carrier_count < 4) {
                // Some plausible frequency
                if (diff < period_30KHz and diff > period_45KHz) {
                    const freq: u16 = @truncate(hw.clock_rate / @as(u32, diff));
                    self.carrier_guesses[self.in_carrier_count] = freq;
                    self.in_carrier_count += 1;

                    // Start of actual pulse started in the past, subtract the difference.
                    self.pulse_start = sample -% (diff * 4);
                } else {
                    self.in_carrier_count = 0;
                }
            } else {
                // End of the pulse, record it
                if (diff > period_20KHz) {
                    self.in_carrier_count = 0;
                    const pulse_end = self.prev_sample;

                    self.pulses.append(.{
                        .high = self.prev_sample -% self.pulse_start,
                        .low = sample -% pulse_end,
                    }) catch |err| {
                        rtt.println("pulses.append: {}", .{err});
                    };
                }
            }

            self.prev_sample = sample;

            self.at = (self.at + 1) & (self.raw_timing.len - 1);
        }

        if (self.pulses.len > 0) {
            const diff = TIM1.CNT.read().CNT -% self.prev_sample;
            const timeout = 65536 / 2;
            if (diff > timeout) {
                if (self.in_carrier_count > 0) {
                    // Finish the last pulse after timeout
                    self.in_carrier_count = 0;
                    self.pulses.append(.{
                        .high = self.prev_sample -% self.pulse_start,
                        .low = 0,
                    }) catch |err| {
                        rtt.println("pulses.append: {}", .{err});
                    };
                }

                rtt.println("{any} Hz", .{self.carrier_guesses});
                for (self.pulses.slice()) |pulse| {
                    rtt.print("{}, {}, ", .{ pulse.high, pulse.low });
                }
                rtt.print_channel_0("\n");

                self.pulses.resize(0) catch unreachable;
            }
        }
    }
};

pub fn enable_receiver(decoder: *Decoder) void {
    DMA.CMAR2.write_raw(@intFromPtr(&decoder.raw_timing));
    DMA.CNDTR2.write_raw(decoder.raw_timing.len);
    DMA.CCR2.modify(.{ .EN = 1 });
    TIM1.DIER.modify(.{ .CC1DE = 1 });
    TIM1.CCER.modify(.{ .CC1E = 1 });
    TIM1.CR1.modify(.{ .CEN = 1 });

    gpio.configure(board.ir_receiver_enabled);

    //RCC.APBENR2.modify(.{ .TIM1EN = 0 });
}

/// Fires when IR transmit is done
pub fn DMA_Channel1_IRQHandler() callconv(.C) void {
    gpio.configure(board.ir_output_off);
    TIM16.CR1.modify(.{ .CEN = 0 });
    TIM17.CR1.modify(.{ .CEN = 0 });
    TIM16.DIER.modify(.{ .CC1DE = 0 }); // Disable DMA request on CC channel 1
    DMA.IFCR.set_others_zero(.{ .CTCIF1 = 1 }); // Clear transfer complete interrupt
    rtt.println("Transmit done", .{});

    RCC.APBENR2.modify(.{
        .TIM16EN = 0,
        .TIM17EN = 0,
    });

    util.write_volatile(&sequence.running, false);
    sequence.run_entry();
}

pub fn is_transmit_busy() bool {
    return DMA.CNDTR1.read_raw() != 0;
}

// Fires when some IR received timings should be processed.
// Received samples are processed after wakeup from WFI
pub fn DMA_Channel2_3_IRQHandler() callconv(.C) void {
    DMA.IFCR.set_others_zero(.{
        // Actually channel 2, not 6 and 5. The svd does not make sense
        .CTCIF5 = 1, // Clear channel 2 transfer complete interrupt
        .CHTIF6 = 1, // Clear channel 2 half transfer complete interrupt
    });
}
