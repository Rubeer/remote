const util = @import("util.zig");
const board = @import("board.zig");
const rtt = @import("rtt.zig");
const hw = @import("hw.zig");
const regs = board.regs;
const ADC = regs.ADC;

const VREFINT_CAL_ADDR: *volatile u16 = @ptrFromInt(0x1FFF75AA);
const VREFINT_CAL_VOLTAGE = 3000;
var vrefint_mult: u32 = undefined;

//const vbat_sample_interval = 60 * 60 * 1000; // 1 hour
const vbat_sample_interval = 1_000; // 1 second

const ADC_State = struct {
    const State = enum(u8) {
        uninitialized = 0,
        calibrating,
        doing_conversion,
        conversion_done,
        disabled,
    };

    state: State,
    vrefint_raw: u16,
    vrefint_mv: u16,
    last_sample_time: u32,
};

var adc_state: ADC_State = undefined;

pub var vrefint_raw: u16 = 0;
pub var vin_mv: u16 = 0;

fn set_state(state: ADC_State.State) void {
    util.write_once(ADC_State.State, &adc_state.state, state);
}

fn get_state() ADC_State.State {
    return util.read_once(ADC_State.State, &adc_state.state);
}

pub fn init() void {
    vrefint_mult = VREFINT_CAL_VOLTAGE * @as(u32, VREFINT_CAL_ADDR.*);
    rtt.println("vrefint_mult: {}", .{vrefint_mult});
    ADC.IER.modify(.{ .EOCALIE = 1 });
    enable_voltage_regulator();
    ADC.CR.modify(.{ .ADCAL = 1 });
    set_state(.calibrating);
    // ADC init continues in adc interrupt (EOCAL)
}

fn enable_voltage_regulator() void {
    ADC.CR.modify(.{ .ADVREGEN = 1 });
    util.busywait(hw.clock_rate / (1_000_000 / 20)); // tADCVREG_SETUP = 20 microseconds
}

fn disable() void {
    // Save a bit more power
    ADC.CCR.modify(.{ .VREFEN = 0 });

    // Assume ADEN will go to 0 due to AUTOFF
    while (ADC.CR.read().ADEN == 1) {
        util.nop();
    }
    ADC.CR.modify(.{ .ADVREGEN = 0 });

    regs.RCC.APBENR2.modify(.{ .ADCEN = 0 });
    set_state(.disabled);
}

fn enable() void {
    regs.RCC.APBENR2.modify(.{ .ADCEN = 1 });
    _ = regs.RCC.APBENR2.read(); // delay for clock startup
    ADC.CCR.modify(.{ .VREFEN = 1 });
    enable_voltage_regulator();
}

pub fn ADC_COMP_IRQHandler() callconv(.C) void {
    const isr = ADC.ISR.read();
    if (isr.EOCAL == 1) {
        rtt.println("EOCAL: {}", .{ADC.DR.read().regularDATA});

        ADC.CFGR1.modify(.{ .AUTOFF = 1 });

        // vREFINT minimum sampling time = 4 us = 64 cycles
        ADC.SMPR.modify(.{ .SMP1 = 0b110 }); // 79.5 cycles
        ADC.CHSELR.write_raw(util.bit(13)); // Enable vREFINT channel
        ADC.CCR.modify(.{ .VREFEN = 1 });
        ADC.IER.modify(.{ .EOCIE = 1 });

        ADC.CR.modify(.{ .ADSTART = 1 });
        set_state(.doing_conversion);
    }

    if (isr.EOC == 1) {
        util.write_once(u16, &vrefint_raw, ADC.DR.read().regularDATA);
        set_state(.conversion_done);
    }

    ADC.ISR.write(isr);
}

pub fn update(ticks: u32) void {
    const state = get_state();

    switch (state) {
        .conversion_done => {
            disable();

            const vref_mv = vrefint_mult / vrefint_raw;
            rtt.println("{} -> {} mV", .{ vrefint_raw, vref_mv });
            adc_state.last_sample_time = ticks;
            set_state(.disabled);
        },
        .disabled => {
            if ((ticks - adc_state.last_sample_time) > vbat_sample_interval) {
                enable();
                ADC.CR.modify(.{ .ADSTART = 1 });
            }
        },
        else => {},
    }

    if (state == .conversion_done) {
        //ADC.CR.modify(.{ .ADSTART = 1 });
    }
}
