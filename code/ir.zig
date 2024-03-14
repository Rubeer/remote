const std = @import("std");
const hw = @import("hw.zig");
const regs = @import("STM32G030.zig").devices.STM32G030.peripherals;
const gpio = @import("gpio.zig");
const board = @import("board.zig");
const rtt = @import("rtt.zig");
const util = @import("util.zig");

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
    .timings = force_constant(.{ 54904, 28289, 6553, 7385, 6554, 21318, 6552, 7384, 6552, 7384, 6554, 7386, 6552, 7385, 6553, 7384, 6552, 7382, 6549, 7384, 6551, 7385, 6552, 7385, 6554, 7384, 6553, 7383, 6553, 21321, 6553, 7385, 6553, 7386, 6555, 7387, 6554, 7385, 6554, 7386, 6554, 7387, 6554, 7386, 6554, 7385, 6554, 7386, 6557, 21326, 6554, 7386, 6555, 7385, 6554, 7385, 6554, 7385, 6554, 7385, 6554, 7385, 6554, 7385, 6555, 7386, 6555, 21323, 6554, 7385, 6553, 21322, 6554, 21321, 6554, 21323, 6555, 21322, 6553, 7387, 6555, 7386, 6554, 21324, 6554, 7385, 6553, 21322, 6553, 21325, 6554, 21325, 6554, 21323, 6553, 7385, 6554, 21327, 6554, 0 }),
};

pub const panasonic_volume_up = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 53595, 28339, 5230, 7421, 5230, 21371, 5231, 7420, 5231, 7421, 5231, 7420, 5232, 7420, 5230, 7419, 5231, 7419, 5230, 7418, 5230, 7420, 5231, 7420, 5229, 7418, 5230, 7420, 5231, 21373, 5232, 7421, 5231, 7421, 5230, 7420, 5229, 7419, 5232, 7420, 5230, 7419, 5229, 7419, 5230, 7420, 5231, 7418, 5229, 21367, 5230, 7417, 5230, 7419, 5231, 7420, 5229, 7418, 5230, 7418, 5229, 7417, 5230, 7420, 5229, 7418, 5230, 7417, 5231, 7419, 5232, 7417, 5231, 7419, 5228, 7420, 5231, 21368, 5230, 7418, 5229, 7418, 5229, 7418, 5230, 7418, 5232, 7420, 5231, 7420, 5231, 7417, 5230, 21368, 5231, 7420, 5232, 21371, 5232, 0 }),
};

pub const panasonic_volume_down = IRCommand{
    .carrier_frequency = 36000,
    .timings = force_constant(.{ 53579, 28308, 5229, 7396, 5229, 21341, 5228, 7398, 5229, 7397, 5229, 7397, 5230, 7397, 5229, 7396, 5229, 7396, 5230, 7398, 5230, 7397, 5229, 7397, 5228, 7396, 5228, 7397, 5228, 21343, 5230, 7398, 5228, 7398, 5230, 7399, 5229, 7397, 5229, 7397, 5229, 7398, 5229, 7399, 5229, 7398, 5231, 7399, 5230, 21345, 5230, 7400, 5230, 7397, 5229, 7397, 5228, 7397, 5229, 7397, 5228, 7397, 5228, 7396, 5229, 7397, 5229, 21340, 5228, 7396, 5228, 7397, 5228, 7397, 5229, 7396, 5228, 21339, 5227, 7395, 5228, 7395, 5228, 21335, 5227, 7395, 5227, 7395, 5229, 7394, 5228, 7398, 5229, 21338, 5228, 7396, 5227, 21336, 5228, 0 }),
};

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
                    self.pulse_start = sample -% diff;
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
