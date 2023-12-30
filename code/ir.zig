const std = @import("std");
const hw = @import("hw.zig");
const regs = @import("STM32G030.zig").devices.STM32G030.peripherals;
const gpio = @import("gpio.zig");
const board = @import("board.zig");
const rtt = @import("rtt.zig");
const TIM16 = regs.TIM16;
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

    // Force the output level to begin active
    var reg = TIM16.CCMR1_Output.read();
    reg.OC1M = .forced_inactive;
    TIM16.CCMR1_Output.write(reg);
    reg.OC1M = .toggle;
    TIM16.CCMR1_Output.write(reg);

    DMA.CCR1.modify(.{ .EN = 1 });
    TIM16.DIER.modify(.{ .CC1DE = 1 }); // Enable DMA request on CC channel 1

    // Switches the output pin from output mode to alternate mode.
    gpio.configure(board.ir_output_on);

    // Enable counters
    TIM16.CR1.modify(.{ .CEN = 1 });
    TIM17.CR1.modify(.{ .CEN = 1 });
}

/// Fires when IR transmit is done
pub fn DMA_Channel1_IRQHandler() callconv(.C) void {
    gpio.configure(board.ir_output_off);
    TIM16.CR1.modify(.{ .CEN = 0 });
    TIM17.CR1.modify(.{ .CEN = 0 });
    DMA.IFCR.set_others_zero(.{ .CTCIF1 = 1 });
    rtt.println("Transmit done", .{});
}
