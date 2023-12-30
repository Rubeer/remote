const svd = @import("STM32G030.zig");
const gpio = @import("gpio.zig");
const board = @import("board.zig");
const util = @import("util.zig");

const regs = svd.devices.STM32G030.peripherals;
const RCC = regs.RCC;
const TIM3 = regs.TIM3;
const TIM16 = regs.TIM16;
const TIM17 = regs.TIM17;
const DMA = regs.DMA;
const DMAMUX = regs.DMAMUX;

const IRQs = util.IRQs;

pub const clock_rate = 16000000;

pub fn init_clock() void {
    // HSI is already enabled startup at 16 MHz.
    // We don't need more than that for this application.
    // Make sure it stays enabled:
    RCC.CR.modify(.{ .HSIKERON = 1 });
}

pub fn init_peripherals() void {
    RCC.APBENR1.modify(.{
        .PWREN = 1,
        .TIM3EN = 1,
    });

    RCC.APBENR2.modify(.{
        .SYSCFGEN = 1,
        .TIM16EN = 1,
        .TIM17EN = 1,
    });

    RCC.AHBENR.modify(.{
        .DMAEN = 1,
    });

    util.enable_irqs(&.{
        IRQs.DMA1_Channel1,
        IRQs.DMA1_Channel2_3,
        IRQs.TIM16,
    });

    // Enable GPIO Clocks
    RCC.IOPENR.modify(.{
        .IOPAEN = 1,
        .IOPBEN = 1,
        .IOPCEN = 1,
    });

    gpio.configure(board.startup_pin_config);

    init_encoder();
    init_ir_transmitter();
}

pub fn read_encoder() i16 {
    return @as(i16, @bitCast(regs.TIM3.CNT.read().CNT_L));
}

fn init_encoder() void {
    TIM3.ARR.write_raw(0xFFFF); // Maxed out counter limit

    TIM3.SMCR.modify(.{
        .SMS = 1, // Encoder mode 1
    });

    // NOTE(robin): Work around limitation in regz
    TIM3.CCMR1_Output.write_raw(@bitCast(svd.CCMR1_Input{
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
    DMAMUX.C0CR.modify(.{ .DMAREQ_ID = 44 }); // TIM16_CH1

    DMA.CCR1.modify(.{
        .EN = 0,
        .DIR = 1, // Memory to peripheral

        .MINC = 1, // enable memory data address increment
        .MSIZE = 1, // 16 bit

        .PINC = 0, // disable periph data address increment
        .PSIZE = 1, // 16 bit
    });

    DMA.CPAR1.write_raw(@intFromPtr(&TIM16.ARR));
    // CMAR1 is set in transmit function

    DMA.CCR1.modify(.{ .TCIE = 1 }); // Enable transfer complete interrupt

    TIM16.ARR.write_raw(10000);
    TIM16.PSC.write_raw(15);
    TIM16.CCMR1_Output.modify(.{ .OC1M = .forced_inactive, .OC1M_2 = 0 }); // Set output to forced inactive
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
}
