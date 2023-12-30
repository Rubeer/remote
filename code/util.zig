const regs = @import("STM32G030.zig").devices.STM32G030.peripherals;
const std = @import("std");

pub fn wfi() void {
    asm volatile ("wfi");
}
pub fn nop() void {
    asm volatile ("nop");
}
pub fn nops(count: u32) void {
    for (0..count) |_|
        asm volatile ("nop");
}

pub const IRQs = enum(u32) {
    //******  STM32G0xxxx specific Interrupt Numbers ****************************************************************//
    WWDG = 0, // Window WatchDog Interrupt                                         */
    RTC_TAMP = 2, // RTC interrupt through the EXTI line 19 & 21                       */
    FLASH = 3, // FLASH global Interrupt                                            */
    RCC = 4, // RCC global Interrupt                                              */
    EXTI0_1 = 5, // EXTI 0 and 1 Interrupts                                           */
    EXTI2_3 = 6, // EXTI Line 2 and 3 Interrupts                                      */
    EXTI4_15 = 7, // EXTI Line 4 to 15 Interrupts                                      */
    DMA1_Channel1 = 9, // DMA1 Channel 1 Interrupt                                          */
    DMA1_Channel2_3 = 10, // DMA1 Channel 2 and Channel 3 Interrupts                           */
    DMA1_Ch4_5_DMAMUX1_OVR = 11, // DMA1 Channel 4 to Channel 5 and DMAMUX1 Overrun Interrupts        */
    ADC1 = 12, // ADC1 Interrupts                                                   */
    TIM1_BRK_UP_TRG_COM = 13, // TIM1 Break, Update, Trigger and Commutation Interrupts            */
    TIM1_CC = 14, // TIM1 Capture Compare Interrupt                                    */
    TIM3 = 16, // TIM3 global Interrupt                                             */
    TIM14 = 19, // TIM14 global Interrupt                                            */
    TIM16 = 21, // TIM16 global Interrupt                                            */
    TIM17 = 22, // TIM17 global Interrupt                                            */
    I2C1 = 23, // I2C1 Interrupt  (combined with EXTI 23)                           */
    I2C2 = 24, // I2C2 Interrupt                                                    */
    SPI1 = 25, // SPI1/I2S1 Interrupt                                               */
    SPI2 = 26, // SPI2 Interrupt                                                    */
    USART1 = 27, // USART1 Interrupt                                                  */
    USART2 = 28, // USART2 Interrupt                                                  */
};

fn combine(comptime nums: []const IRQs) u32 {
    var combined: u32 = 0;
    inline for (nums) |num| {
        combined |= @as(u32, 1) << @intFromEnum(num);
    }
    return combined;
}

pub fn enable_irqs(comptime nums: []const IRQs) void {
    asm volatile ("" ::: "memory");
    regs.NVIC.ISER.write_raw(combine(nums));
    asm volatile ("" ::: "memory");
}
pub fn disable_irqs(comptime nums: []const IRQs) void {
    regs.NVIC.ICER.write_raw(combine(nums));
}
