const std = @import("std");
const gpio = @import("gpio.zig");
const board = @import("board.zig");
const util = @import("util.zig");
const keys = @import("keymatrix.zig");

const regs = board.regs;
const RCC = regs.RCC;
const TIM1 = regs.TIM1;
const TIM3 = regs.TIM3;
const TIM14 = regs.TIM14;
const TIM16 = regs.TIM16;
const TIM17 = regs.TIM17;
const DMA = regs.DMA;
const DMAMUX = regs.DMAMUX;
const ADC = regs.ADC;

const IRQs = util.IRQs;

pub const clock_rate = 16000000;
const systick_interval_ms = 10;

var ticks_ms: u32 = 0;
pub fn SysTick_Handler() callconv(.C) void {
    ticks_ms +%= systick_interval_ms;
}

pub fn get_ticks() u32 {
    return util.read_volatile(&ticks_ms);
}

pub fn init_clock() void {
    // HSI is already enabled at startup, 16 MHz.
    // We don't need more than that for this application.

    regs.PWR.CR1.modify(.{ .VOS = 0b01 }); // Range 1 (low power)
}

pub fn init_peripherals() void {
    RCC.APBENR1.modify(.{
        .PWREN = 1,
        .TIM3EN = 1,
    });

    RCC.APBENR2.modify(.{
        .SYSCFGEN = 1,
        .TIM1EN = 1,
        .TIM14EN = 1,
        .TIM16EN = 1,
        .TIM17EN = 1,
        .ADCEN = 1,
    });

    RCC.AHBENR.modify(.{
        .DMAEN = 1,
    });

    // Enable GPIO Clocks
    RCC.IOPENR.modify(.{
        .IOPAEN = 1,
        .IOPBEN = 1,
        .IOPCEN = 1,
    });

    util.enable_irqs(&.{
        IRQs.DMA1_Channel1,
        IRQs.DMA1_Channel2_3,
        IRQs.TIM16,
        IRQs.TIM14,
        IRQs.ADC1,
    });

    gpio.configure(board.startup_pin_config);

    regs.EXTI.FTSR1.write_raw(keys.cols_mask);
    regs.EXTI.IMR1.write_raw(keys.cols_mask);

    // Enable SysTick 10ms interval
    regs.STK.RVR.write_raw(((clock_rate * systick_interval_ms) / 1000) - 1);
    regs.STK.CSR.modify(.{
        .ENABLE = 1,
        .CLKSOURCE = 1,
        .TICKINT = 1,
    });

    //init_vbat_measure();
    init_encoder();
    init_ir_transmitter();
    init_ir_receiver();
    init_led_timer();
}

pub fn read_encoder() i16 {
    const signed: i16 = @bitCast(TIM3.CNT.read().CNT_L);
    const div2 = @divTrunc(signed, 2);
    return -div2;
}

fn init_encoder() void {
    TIM3.ARR.write_raw(0xFFFF); // Maxed out counter limit

    TIM3.SMCR.modify(.{ .SMS = 1 }); // Encoder mode 1

    // NOTE(robin): Work around limitation in regz
    TIM3.CCMR1_Output.write_raw(@bitCast(regs.CCMR1_Input{
        .CC1S = 1, // Input TI1,
        .IC1PSC = 0,
        .IC1F = 0b1111, // Strongest filter
        .CC2S = 1, // Input TI2
        .IC2PSC = 0,
        .IC2F = 15,
    }));

    TIM3.CR1.modify(.{
        .UIFREMAP = 0, // No remapping
        .CKD = 0, // No clock division
        .ARPE = 0, // Disable auto-reload preload
        .CMS = 0, // Edge-aligned mode
        .DIR = 0, // Upcounter
        .OPM = 0, // Disable one-pulse
        .URS = 1, // Fewest event sources
        .UDIS = 1, // Disable update event
        .CEN = 1, // Counter enabled
    });
}

fn init_ir_transmitter() void {
    DMAMUX.C0CR.modify(.{ .DMAREQ_ID = 44 }); // 44 = TIM16_CH1

    DMA.CCR1.modify(.{
        .EN = 0,
        .DIR = 1, // Memory to peripheral

        .MINC = 1, // enable memory data address increment
        .MSIZE = 1, // 16 bit

        .PINC = 0, // disable peripheral data address increment
        .PSIZE = 1, // 16 bit

        .TCIE = 1, // Enable transfer complete interrupt
    });

    DMA.CPAR1.write_raw(@intFromPtr(&TIM16.ARR));
    // CMAR1 is set in transmit function

    TIM16.ARR.write_raw(10000);
    TIM16.PSC.write_raw(0);
    TIM16.CCMR1_Output.modify(.{
        .OC1M = .forced_inactive,
        .OC1M_2 = 0,
    });
    TIM16.CR2.modify(.{ .CCDS = 0 }); // DMA request to CC mode

    // Enable outputs
    TIM16.BDTR.modify(.{ .MOE = 1 });
    TIM17.BDTR.modify(.{ .MOE = 1 });

    TIM17.ARR.write_raw(420); // Default carrier: 38 KHz
    TIM17.CCR1.write_raw(420 / 3); // 1/3 duty cycle
    TIM17.PSC.write_raw(0);
    TIM17.CCMR1_Output.modify(.{
        .OC1M = .pwm_1,
        .OC1PE = 1, // OC preload enabled
    });

    regs.SYSCFG.CFGR1.modify(.{
        .IR_POL = 1,
        .IR_MOD = 0, // TIM16
    });

    // Re-enabled once we need to transmit
    RCC.APBENR2.modify(.{
        .TIM16EN = 0,
        .TIM17EN = 0,
    });
}

fn init_ir_receiver() void {
    DMAMUX.C1CR.modify(.{ .DMAREQ_ID = 20 }); // 20 = TIM1_CH1

    DMA.CCR2.modify(.{
        .EN = 0,
        .DIR = 0, // Peripheral to memory

        .CIRC = 1, // Enable circular mode

        .MINC = 1, // enable memory data address increment
        .MSIZE = 1, // 16 bit

        .PINC = 0, // disable peripheral data address increment
        .PSIZE = 1, // 16 bit

        .TCIE = 1, // Enable transfer complete interrupt
        .HTIE = 1, // Enable half transfer complete interrupt
    });

    DMA.CPAR2.write_raw(@intFromPtr(&TIM1.CCR1));

    TIM1.ARR.write_raw(65535);

    // NOTE(robin): Work around limitation in regz
    var reg = std.mem.zeroes(regs.CCMR1_Input);
    reg.CC1S = 1; // Input TI1
    TIM1.CCMR1_Output.write_raw(@bitCast(reg));
}

fn init_led_timer() void {
    TIM14.PSC.write_raw(15); // 1 us resolution
    TIM14.CR1.modify(.{ .OPM = 1 }); // One pulse mode
    TIM14.DIER.modify(.{ .UIE = 1 }); // Enable update interrupt
}

fn init_vbat_measure() void {
    @import("adc.zig").init();
}

/// Go to stop 1 mode, wake up from EXTI handlers
/// (Any button or encoder input)
pub fn go_to_sleep() void {
    util.disable_irqs(&.{.TIM14});
    gpio.configure(board.sleep_config);
    regs.EXTI.FPR1.write_raw(keys.cols_mask);
    regs.PWR.CR1.modify(.{ .LPMS = 0b001 });
    regs.SCB.SCR.modify(.{ .SLEEPDEEP = 1 });
    util.wfi();
    regs.SCB.SCR.modify(.{ .SLEEPDEEP = 0 });
}
