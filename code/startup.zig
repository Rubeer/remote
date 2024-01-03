const std = @import("std");
const rtt = @import("rtt.zig");
const hw = @import("hw.zig");
const ir = @import("ir.zig");
const keys = @import("keymatrix.zig");

extern const _data_start_in_flash: u8;
extern const _data_start_in_ram: u8;
extern const _data_end_in_ram: u8;
extern const _bss_start: u8;
extern const _bss_end: u8;

extern fn _end_of_stack() void; // This is not really a function, but zig has trouble putting a u32 in the vector table

export fn Reset_Handler() callconv(.C) noreturn {
    hw.init_clock();

    {
        const bss_start: [*]u8 = @ptrCast(&_bss_start);
        const bss_end: [*]u8 = @ptrCast(&_bss_end);
        const bss_len = @intFromPtr(bss_end) - @intFromPtr(bss_start);

        @memset(bss_start[0..bss_len], 0);
    }

    {
        const data_start: [*]u8 = @ptrCast(&_data_start_in_ram);
        const data_end: [*]u8 = @ptrCast(&_data_end_in_ram);
        const data_len = @intFromPtr(data_end) - @intFromPtr(data_start);
        const data_source: [*]const u8 = @ptrCast(&_data_start_in_flash);

        @memcpy(data_start[0..data_len], data_source[0..data_len]);
    }
    hw.init_peripherals();

    @import("main.zig").main();
}

pub fn Default_Handler() callconv(.C) void {
    rtt.println("Default_Handler!\n", .{});
    @breakpoint();
}

fn HardFault_Handler() callconv(.C) void {
    rtt.println("HardFault! {x}\n", .{@returnAddress()});
    @breakpoint();
}

const svd = @import("STM32G030.zig");
export const vector_table linksection(".vector_table") = svd.devices.STM32G030.VectorTable{
    .initial_stack_pointer = _end_of_stack,
    .Reset = Reset_Handler,
    .HardFault = HardFault_Handler,
    .DMA_Channel1 = ir.DMA_Channel1_IRQHandler,
    .DMA_Channel2_3 = ir.DMA_Channel2_3_IRQHandler,
    .TIM14 = keys.TIM14_IRQHandler,
    .SysTick = hw.SysTick_Handler,
};
