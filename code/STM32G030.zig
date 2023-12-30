const mmio = @import("mmio.zig");
const startup = @import("startup.zig");

/// Work around limitation in regz
pub const CCMR1_Input = packed struct(u32) {
    ///  Capture/Compare 1 selection
    CC1S: u2,
    /// Input capture 1 prescaler
    IC1PSC: u2,
    /// Input capture 1 filter
    IC1F: u4,
    /// Capture/compare 2 selection
    CC2S: u2,
    /// Input capture 2 prescaler
    IC2PSC: u2,
    /// Input capture 2 filter
    IC2F: u4,
    padding: u16 = 0,
};

pub const devices = struct {
    ///  STM32G030
    pub const STM32G030 = struct {
        pub const properties = struct {
            pub const @"cpu.endian" = "little";
            pub const @"cpu.mpuPresent" = "true";
            pub const @"cpu.revision" = "r0p1";
            pub const @"cpu.name" = "CM0";
            pub const @"cpu.nvicPrioBits" = "4";
            pub const @"cpu.vendorSystickConfig" = "false";
            pub const @"cpu.fpuPresent" = "false";
        };

        pub const VectorTable = extern struct {
            const Handler = *const fn () callconv(.C) void;
            const unhandled = startup.Default_Handler;

            initial_stack_pointer: Handler,
            Reset: *const fn () callconv(.C) noreturn,
            NMI: Handler = unhandled,
            HardFault: Handler = unhandled,
            reserved2: [7]u32 = undefined,
            SVCall: Handler = unhandled,
            reserved10: [2]u32 = undefined,
            PendSV: Handler = unhandled,
            SysTick: Handler = unhandled,
            ///  Window watchdog interrupt
            WWDG: Handler = unhandled,
            ///  Power voltage detector interrupt
            PVD: Handler = unhandled,
            ///  RTC and TAMP interrupts
            RTC_TAMP: Handler = unhandled,
            ///  Flash global interrupt
            FLASH: Handler = unhandled,
            ///  RCC global interrupt
            RCC: Handler = unhandled,
            ///  EXTI line 0 & 1 interrupt
            EXTI0_1: Handler = unhandled,
            ///  EXTI line 2 & 3 interrupt
            EXTI2_3: Handler = unhandled,
            ///  EXTI line 4 to 15 interrupt
            EXTI4_15: Handler = unhandled,
            reserved22: [1]u32 = undefined,
            ///  DMA channel 1 interrupt
            DMA_Channel1: Handler = unhandled,
            ///  DMA channel 2 & 3 interrupts
            DMA_Channel2_3: Handler = unhandled,
            ///  DMA channel 4, 5, 6 & 7 and DMAMUX
            DMA_Channel4_5_6_7: Handler = unhandled,
            ///  ADC and COMP interrupts
            ADC_COMP: Handler = unhandled,
            ///  TIM1 break, update, trigger
            TIM1_BRK_UP_TRG_COM: Handler = unhandled,
            ///  TIM1 Capture Compare interrupt
            TIM1_CC: Handler = unhandled,
            ///  TIM2 global interrupt
            TIM2: Handler = unhandled,
            ///  TIM3 global interrupt
            TIM3: Handler = unhandled,
            reserved31: [2]u32 = undefined,
            ///  TIM14 global interrupt
            TIM14: Handler = unhandled,
            reserved34: [1]u32 = undefined,
            ///  TIM16 global interrupt
            TIM16: Handler = unhandled,
            ///  TIM17 global interrupt
            TIM17: Handler = unhandled,
            ///  I2C1 global interrupt
            I2C1: Handler = unhandled,
            ///  I2C2 global interrupt
            I2C2: Handler = unhandled,
            ///  SPI1 global interrupt
            SPI1: Handler = unhandled,
            ///  SPI2 global interrupt
            SPI2: Handler = unhandled,
            ///  USART1 global interrupt
            USART1: Handler = unhandled,
            ///  USART2 global interrupt
            USART2: Handler = unhandled,
            reserved43: [1]u32 = undefined,
            ///  CEC global interrupt
            CEC: Handler = unhandled,
        };

        pub const peripherals = struct {
            ///  General-purpose-timers
            pub const TIM2: *volatile types.peripherals.TIM2 = @ptrFromInt(0x40000000);
            ///  General-purpose-timers
            pub const TIM3: *volatile types.peripherals.TIM2 = @ptrFromInt(0x40000400);
            ///  General purpose timers
            pub const TIM14: *volatile types.peripherals.TIM14 = @ptrFromInt(0x40002000);
            ///  Real-time clock
            pub const RTC: *volatile types.peripherals.RTC = @ptrFromInt(0x40002800);
            ///  System window watchdog
            pub const WWDG: *volatile types.peripherals.WWDG = @ptrFromInt(0x40002c00);
            ///  Independent watchdog
            pub const IWDG: *volatile types.peripherals.IWDG = @ptrFromInt(0x40003000);
            ///  Serial peripheral interface/Inter-IC sound
            pub const SPI2: *volatile types.peripherals.SPI1 = @ptrFromInt(0x40003800);
            ///  Universal synchronous asynchronous receiver transmitter
            pub const USART2: *volatile types.peripherals.USART1 = @ptrFromInt(0x40004400);
            ///  Inter-integrated circuit
            pub const I2C1: *volatile types.peripherals.I2C1 = @ptrFromInt(0x40005400);
            ///  Inter-integrated circuit
            pub const I2C2: *volatile types.peripherals.I2C1 = @ptrFromInt(0x40005800);
            ///  Power control
            pub const PWR: *volatile types.peripherals.PWR = @ptrFromInt(0x40007000);
            ///  Tamper and backup registers
            pub const TAMP: *volatile types.peripherals.TAMP = @ptrFromInt(0x4000b000);
            ///  System configuration controller
            pub const SYSCFG: *volatile types.peripherals.SYSCFG = @ptrFromInt(0x40010000);
            ///  System configuration controller
            pub const VREFBUF: *volatile types.peripherals.VREFBUF = @ptrFromInt(0x40010030);
            ///  System configuration controller
            pub const SYSCFG_ITLINE: *volatile types.peripherals.SYSCFG_ITLINE = @ptrFromInt(0x40010080);
            ///  Analog to Digital Converter instance 1
            pub const ADC: *volatile types.peripherals.ADC = @ptrFromInt(0x40012400);
            ///  Advanced-timers
            pub const TIM1: *volatile types.peripherals.TIM1 = @ptrFromInt(0x40012c00);
            ///  Serial peripheral interface/Inter-IC sound
            pub const SPI1: *volatile types.peripherals.SPI1 = @ptrFromInt(0x40013000);
            ///  Universal synchronous asynchronous receiver transmitter
            pub const USART1: *volatile types.peripherals.USART1 = @ptrFromInt(0x40013800);
            ///  General purpose timers
            pub const TIM16: *volatile types.peripherals.TIM16 = @ptrFromInt(0x40014400);
            ///  General purpose timers
            pub const TIM17: *volatile types.peripherals.TIM16 = @ptrFromInt(0x40014800);
            ///  MCU debug component
            pub const DBG: *volatile types.peripherals.DBG = @ptrFromInt(0x40015800);
            ///  DMA controller
            pub const DMA: *volatile types.peripherals.DMA = @ptrFromInt(0x40020000);
            ///  DMAMUX
            pub const DMAMUX: *volatile types.peripherals.DMAMUX = @ptrFromInt(0x40020800);
            ///  Reset and clock control
            pub const RCC: *volatile types.peripherals.RCC = @ptrFromInt(0x40021000);
            ///  External interrupt/event controller
            pub const EXTI: *volatile types.peripherals.EXTI = @ptrFromInt(0x40021800);
            ///  Flash
            pub const FLASH: *volatile types.peripherals.FLASH = @ptrFromInt(0x40022000);
            ///  Cyclic redundancy check calculation unit
            pub const CRC: *volatile types.peripherals.CRC = @ptrFromInt(0x40023000);
            ///  General-purpose I/Os
            pub const GPIOA: *volatile types.peripherals.GPIOA = @ptrFromInt(0x50000000);
            ///  General-purpose I/Os
            pub const GPIOB: *volatile types.peripherals.GPIOB = @ptrFromInt(0x50000400);
            ///  General-purpose I/Os
            pub const GPIOC: *volatile types.peripherals.GPIOB = @ptrFromInt(0x50000800);
            ///  General-purpose I/Os
            pub const GPIOD: *volatile types.peripherals.GPIOB = @ptrFromInt(0x50000c00);
            ///  General-purpose I/Os
            pub const GPIOF: *volatile types.peripherals.GPIOB = @ptrFromInt(0x50001400);
            ///  System control block ACTLR
            pub const SCB_ACTRL: *volatile types.peripherals.SCB_ACTRL = @ptrFromInt(0xe000e008);
            ///  SysTick timer
            pub const STK: *volatile types.peripherals.STK = @ptrFromInt(0xe000e010);
            ///  Nested Vectored Interrupt Controller
            pub const NVIC: *volatile types.peripherals.NVIC = @ptrFromInt(0xe000e100);
            ///  System control block
            pub const SCB: *volatile types.peripherals.SCB = @ptrFromInt(0xe000ed00);
            ///  Floating point unit CPACR
            pub const FPU_CPACR: *volatile types.peripherals.FPU_CPACR = @ptrFromInt(0xe000ed88);
            ///  Memory protection unit
            pub const MPU: *volatile types.peripherals.MPU = @ptrFromInt(0xe000ed90);
            ///  Nested vectored interrupt controller
            pub const NVIC_STIR: *volatile types.peripherals.NVIC_STIR = @ptrFromInt(0xe000ef00);
            ///  Floting point unit
            pub const FPU: *volatile types.peripherals.FPU = @ptrFromInt(0xe000ef34);
        };
    };
};

pub const types = struct {
    pub const peripherals = struct {
        ///  Independent watchdog
        pub const IWDG = extern struct {
            ///  Key register
            KR: mmio.Mmio(packed struct(u32) {
                ///  Key value (write only, read 0x0000)
                KEY: u16,
                padding: u16,
            }),
            ///  Prescaler register
            PR: mmio.Mmio(packed struct(u32) {
                ///  Prescaler divider
                PR: u3,
                padding: u29,
            }),
            ///  Reload register
            RLR: mmio.Mmio(packed struct(u32) {
                ///  Watchdog counter reload value
                RL: u12,
                padding: u20,
            }),
            ///  Status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  Watchdog prescaler value update
                PVU: u1,
                ///  Watchdog counter reload value update
                RVU: u1,
                ///  Watchdog counter window value update
                WVU: u1,
                padding: u29,
            }),
            ///  Window register
            WINR: mmio.Mmio(packed struct(u32) {
                ///  Watchdog counter window value
                WIN: u12,
                padding: u20,
            }),
        };

        ///  System window watchdog
        pub const WWDG = extern struct {
            ///  Control register
            CR: mmio.Mmio(packed struct(u32) {
                ///  7-bit counter (MSB to LSB)
                T: u7,
                ///  Activation bit
                WDGA: u1,
                padding: u24,
            }),
            ///  Configuration register
            CFR: mmio.Mmio(packed struct(u32) {
                ///  7-bit window value
                W: u7,
                reserved9: u2,
                ///  Early wakeup interrupt
                EWI: u1,
                reserved11: u1,
                ///  Timer base
                WDGTB: u3,
                padding: u18,
            }),
            ///  Status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  Early wakeup interrupt flag
                EWIF: u1,
                padding: u31,
            }),
        };

        ///  Flash
        pub const FLASH = extern struct {
            ///  Access control register
            ACR: mmio.Mmio(packed struct(u32) {
                ///  Latency
                LATENCY: u3,
                reserved8: u5,
                ///  Prefetch enable
                PRFTEN: u1,
                ///  Instruction cache enable
                ICEN: u1,
                reserved11: u1,
                ///  Instruction cache reset
                ICRST: u1,
                reserved16: u4,
                ///  Flash User area empty
                EMPTY: u1,
                reserved18: u1,
                ///  Debug access software enable
                DBG_SWEN: u1,
                padding: u13,
            }),
            reserved8: [4]u8,
            ///  Flash key register
            KEYR: mmio.Mmio(packed struct(u32) {
                ///  KEYR
                KEYR: u32,
            }),
            ///  Option byte key register
            OPTKEYR: mmio.Mmio(packed struct(u32) {
                ///  Option byte key
                OPTKEYR: u32,
            }),
            ///  Status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  End of operation
                EOP: u1,
                ///  Operation error
                OPERR: u1,
                reserved3: u1,
                ///  Programming error
                PROGERR: u1,
                ///  Write protected error
                WRPERR: u1,
                ///  Programming alignment error
                PGAERR: u1,
                ///  Size error
                SIZERR: u1,
                ///  Programming sequence error
                PGSERR: u1,
                ///  Fast programming data miss error
                MISERR: u1,
                ///  Fast programming error
                FASTERR: u1,
                reserved14: u4,
                ///  PCROP read error
                RDERR: u1,
                ///  Option and Engineering bits loading validity error
                OPTVERR: u1,
                ///  Busy
                BSY: u1,
                reserved18: u1,
                ///  Programming or erase configuration busy.
                CFGBSY: u1,
                padding: u13,
            }),
            ///  Flash control register
            CR: mmio.Mmio(packed struct(u32) {
                ///  Programming
                PG: u1,
                ///  Page erase
                PER: u1,
                ///  Mass erase
                MER: u1,
                ///  Page number
                PNB: u6,
                reserved16: u7,
                ///  Start
                STRT: u1,
                ///  Options modification start
                OPTSTRT: u1,
                ///  Fast programming
                FSTPG: u1,
                reserved24: u5,
                ///  End of operation interrupt enable
                EOPIE: u1,
                ///  Error interrupt enable
                ERRIE: u1,
                ///  PCROP read error interrupt enable
                RDERRIE: u1,
                ///  Force the option byte loading
                OBL_LAUNCH: u1,
                ///  Securable memory area protection enable
                SEC_PROT: u1,
                reserved30: u1,
                ///  Options Lock
                OPTLOCK: u1,
                ///  FLASH_CR Lock
                LOCK: u1,
            }),
            ///  Flash ECC register
            ECCR: mmio.Mmio(packed struct(u32) {
                ///  ECC fail address
                ADDR_ECC: u14,
                reserved20: u6,
                ///  ECC fail for Corrected ECC Error or Double ECC Error in info block
                SYSF_ECC: u1,
                reserved24: u3,
                ///  ECC correction interrupt enable
                ECCIE: u1,
                reserved30: u5,
                ///  ECC correction
                ECCC: u1,
                ///  ECC detection
                ECCD: u1,
            }),
            reserved32: [4]u8,
            ///  Flash option register
            OPTR: mmio.Mmio(packed struct(u32) {
                ///  Read protection level
                RDP: u8,
                ///  BOR reset Level
                BOREN: u1,
                ///  These bits contain the VDD supply level threshold that activates the reset
                BORF_LEV: u2,
                ///  These bits contain the VDD supply level threshold that releases the reset.
                BORR_LEV: u2,
                ///  nRST_STOP
                nRST_STOP: u1,
                ///  nRST_STDBY
                nRST_STDBY: u1,
                ///  nRSTS_HDW
                nRSTS_HDW: u1,
                ///  Independent watchdog selection
                IDWG_SW: u1,
                ///  Independent watchdog counter freeze in Stop mode
                IWDG_STOP: u1,
                ///  Independent watchdog counter freeze in Standby mode
                IWDG_STDBY: u1,
                ///  Window watchdog selection
                WWDG_SW: u1,
                reserved22: u2,
                ///  SRAM parity check control
                RAM_PARITY_CHECK: u1,
                reserved24: u1,
                ///  nBOOT_SEL
                nBOOT_SEL: u1,
                ///  Boot configuration
                nBOOT1: u1,
                ///  nBOOT0 option bit
                nBOOT0: u1,
                ///  NRST_MODE
                NRST_MODE: u2,
                ///  Internal reset holder enable bit
                IRHEN: u1,
                padding: u2,
            }),
            ///  Flash PCROP zone A Start address register
            PCROP1ASR: mmio.Mmio(packed struct(u32) {
                ///  PCROP1A area start offset
                PCROP1A_STRT: u8,
                padding: u24,
            }),
            ///  Flash PCROP zone A End address register
            PCROP1AER: mmio.Mmio(packed struct(u32) {
                ///  PCROP1A area end offset
                PCROP1A_END: u8,
                reserved31: u23,
                ///  PCROP area preserved when RDP level decreased
                PCROP_RDP: u1,
            }),
            ///  Flash WRP area A address register
            WRP1AR: mmio.Mmio(packed struct(u32) {
                ///  WRP area A start offset
                WRP1A_STRT: u6,
                reserved16: u10,
                ///  WRP area A end offset
                WRP1A_END: u6,
                padding: u10,
            }),
            ///  Flash WRP area B address register
            WRP1BR: mmio.Mmio(packed struct(u32) {
                ///  WRP area B start offset
                WRP1B_STRT: u6,
                reserved16: u10,
                ///  WRP area B end offset
                WRP1B_END: u6,
                padding: u10,
            }),
            ///  Flash PCROP zone B Start address register
            PCROP1BSR: mmio.Mmio(packed struct(u32) {
                ///  PCROP1B area start offset
                PCROP1B_STRT: u8,
                padding: u24,
            }),
            ///  Flash PCROP zone B End address register
            PCROP1BER: mmio.Mmio(packed struct(u32) {
                ///  PCROP1B area end offset
                PCROP1B_END: u8,
                padding: u24,
            }),
            reserved128: [68]u8,
            ///  Flash Security register
            SECR: mmio.Mmio(packed struct(u32) {
                ///  Securable memory area size
                SEC_SIZE: u7,
                reserved16: u9,
                ///  used to force boot from user area
                BOOT_LOCK: u1,
                padding: u15,
            }),
        };

        ///  Reset and clock control
        pub const RCC = extern struct {
            ///  Clock control register
            CR: mmio.Mmio(packed struct(u32) {
                reserved8: u8,
                ///  HSI16 clock enable
                HSION: u1,
                ///  HSI16 always enable for peripheral kernels
                HSIKERON: u1,
                ///  HSI16 clock ready flag
                HSIRDY: u1,
                ///  HSI16 clock division factor
                HSIDIV: u3,
                reserved16: u2,
                ///  HSE clock enable
                HSEON: u1,
                ///  HSE clock ready flag
                HSERDY: u1,
                ///  HSE crystal oscillator bypass
                HSEBYP: u1,
                ///  Clock security system enable
                CSSON: u1,
                reserved24: u4,
                ///  PLL enable
                PLLON: u1,
                ///  PLL clock ready flag
                PLLRDY: u1,
                padding: u6,
            }),
            ///  Internal clock sources calibration register
            ICSCR: mmio.Mmio(packed struct(u32) {
                ///  HSI16 clock calibration
                HSICAL: u8,
                ///  HSI16 clock trimming
                HSITRIM: u7,
                padding: u17,
            }),
            ///  Clock configuration register
            CFGR: mmio.Mmio(packed struct(u32) {
                ///  System clock switch
                SW: u3,
                ///  System clock switch status
                SWS: u3,
                reserved8: u2,
                ///  AHB prescaler
                HPRE: u4,
                ///  APB prescaler
                PPRE: u3,
                reserved24: u9,
                ///  Microcontroller clock output
                MCOSEL: u3,
                reserved28: u1,
                ///  Microcontroller clock output prescaler
                MCOPRE: u3,
                padding: u1,
            }),
            ///  PLL configuration register
            PLLSYSCFGR: mmio.Mmio(packed struct(u32) {
                ///  PLL input clock source
                PLLSRC: u2,
                reserved4: u2,
                ///  Division factor M of the PLL input clock divider
                PLLM: u3,
                reserved8: u1,
                ///  PLL frequency multiplication factor N
                PLLN: u7,
                reserved16: u1,
                ///  PLLPCLK clock output enable
                PLLPEN: u1,
                ///  PLL VCO division factor P for PLLPCLK clock output
                PLLP: u5,
                reserved24: u2,
                ///  PLLQCLK clock output enable
                PLLQEN: u1,
                ///  PLL VCO division factor Q for PLLQCLK clock output
                PLLQ: u3,
                ///  PLLRCLK clock output enable
                PLLREN: u1,
                ///  PLL VCO division factor R for PLLRCLK clock output
                PLLR: u3,
            }),
            reserved24: [8]u8,
            ///  Clock interrupt enable register
            CIER: mmio.Mmio(packed struct(u32) {
                ///  LSI ready interrupt enable
                LSIRDYIE: u1,
                ///  LSE ready interrupt enable
                LSERDYIE: u1,
                reserved3: u1,
                ///  HSI ready interrupt enable
                HSIRDYIE: u1,
                ///  HSE ready interrupt enable
                HSERDYIE: u1,
                ///  PLL ready interrupt enable
                PLLSYSRDYIE: u1,
                padding: u26,
            }),
            ///  Clock interrupt flag register
            CIFR: mmio.Mmio(packed struct(u32) {
                ///  LSI ready interrupt flag
                LSIRDYF: u1,
                ///  LSE ready interrupt flag
                LSERDYF: u1,
                reserved3: u1,
                ///  HSI ready interrupt flag
                HSIRDYF: u1,
                ///  HSE ready interrupt flag
                HSERDYF: u1,
                ///  PLL ready interrupt flag
                PLLSYSRDYF: u1,
                reserved8: u2,
                ///  Clock security system interrupt flag
                CSSF: u1,
                ///  LSE Clock security system interrupt flag
                LSECSSF: u1,
                padding: u22,
            }),
            ///  Clock interrupt clear register
            CICR: mmio.Mmio(packed struct(u32) {
                ///  LSI ready interrupt clear
                LSIRDYC: u1,
                ///  LSE ready interrupt clear
                LSERDYC: u1,
                reserved3: u1,
                ///  HSI ready interrupt clear
                HSIRDYC: u1,
                ///  HSE ready interrupt clear
                HSERDYC: u1,
                ///  PLL ready interrupt clear
                PLLSYSRDYC: u1,
                reserved8: u2,
                ///  Clock security system interrupt clear
                CSSC: u1,
                ///  LSE Clock security system interrupt clear
                LSECSSC: u1,
                padding: u22,
            }),
            ///  GPIO reset register
            IOPRSTR: mmio.Mmio(packed struct(u32) {
                ///  I/O port A reset
                IOPARST: u1,
                ///  I/O port B reset
                IOPBRST: u1,
                ///  I/O port C reset
                IOPCRST: u1,
                ///  I/O port D reset
                IOPDRST: u1,
                reserved5: u1,
                ///  I/O port F reset
                IOPFRST: u1,
                padding: u26,
            }),
            ///  AHB peripheral reset register
            AHBRSTR: mmio.Mmio(packed struct(u32) {
                ///  DMA1 reset
                DMARST: u1,
                reserved8: u7,
                ///  FLITF reset
                FLASHRST: u1,
                reserved12: u3,
                ///  CRC reset
                CRCRST: u1,
                padding: u19,
            }),
            ///  APB peripheral reset register 1
            APBRSTR1: mmio.Mmio(packed struct(u32) {
                ///  TIM2 timer reset
                TIM2RST: u1,
                ///  TIM3 timer reset
                TIM3RST: u1,
                reserved14: u12,
                ///  SPI2 reset
                SPI2RST: u1,
                reserved17: u2,
                ///  USART2 reset
                USART2RST: u1,
                reserved21: u3,
                ///  I2C1 reset
                I2C1RST: u1,
                ///  I2C2 reset
                I2C2RST: u1,
                reserved27: u4,
                ///  Debug support reset
                DBGRST: u1,
                ///  Power interface reset
                PWRRST: u1,
                padding: u3,
            }),
            ///  APB peripheral reset register 2
            APBRSTR2: mmio.Mmio(packed struct(u32) {
                ///  SYSCFG, COMP and VREFBUF reset
                SYSCFGRST: u1,
                reserved11: u10,
                ///  TIM1 timer reset
                TIM1RST: u1,
                ///  SPI1 reset
                SPI1RST: u1,
                reserved14: u1,
                ///  USART1 reset
                USART1RST: u1,
                ///  TIM14 timer reset
                TIM14RST: u1,
                reserved17: u1,
                ///  TIM16 timer reset
                TIM16RST: u1,
                ///  TIM17 timer reset
                TIM17RST: u1,
                reserved20: u1,
                ///  ADC reset
                ADCRST: u1,
                padding: u11,
            }),
            ///  GPIO clock enable register
            IOPENR: mmio.Mmio(packed struct(u32) {
                ///  I/O port A clock enable
                IOPAEN: u1,
                ///  I/O port B clock enable
                IOPBEN: u1,
                ///  I/O port C clock enable
                IOPCEN: u1,
                ///  I/O port D clock enable
                IOPDEN: u1,
                reserved5: u1,
                ///  I/O port F clock enable
                IOPFEN: u1,
                padding: u26,
            }),
            ///  AHB peripheral clock enable register
            AHBENR: mmio.Mmio(packed struct(u32) {
                ///  DMA clock enable
                DMAEN: u1,
                reserved8: u7,
                ///  Flash memory interface clock enable
                FLASHEN: u1,
                reserved12: u3,
                ///  CRC clock enable
                CRCEN: u1,
                padding: u19,
            }),
            ///  APB peripheral clock enable register 1
            APBENR1: mmio.Mmio(packed struct(u32) {
                ///  TIM2 timer clock enable
                TIM2EN: u1,
                ///  TIM3 timer clock enable
                TIM3EN: u1,
                reserved10: u8,
                ///  RTC APB clock enable
                RTCAPBEN: u1,
                ///  WWDG clock enable
                WWDGEN: u1,
                reserved14: u2,
                ///  SPI2 clock enable
                SPI2EN: u1,
                reserved17: u2,
                ///  USART2 clock enable
                USART2EN: u1,
                reserved21: u3,
                ///  I2C1 clock enable
                I2C1EN: u1,
                ///  I2C2 clock enable
                I2C2EN: u1,
                reserved27: u4,
                ///  Debug support clock enable
                DBGEN: u1,
                ///  Power interface clock enable
                PWREN: u1,
                padding: u3,
            }),
            ///  APB peripheral clock enable register 2
            APBENR2: mmio.Mmio(packed struct(u32) {
                ///  SYSCFG, COMP and VREFBUF clock enable
                SYSCFGEN: u1,
                reserved11: u10,
                ///  TIM1 timer clock enable
                TIM1EN: u1,
                ///  SPI1 clock enable
                SPI1EN: u1,
                reserved14: u1,
                ///  USART1 clock enable
                USART1EN: u1,
                ///  TIM14 timer clock enable
                TIM14EN: u1,
                reserved17: u1,
                ///  TIM16 timer clock enable
                TIM16EN: u1,
                ///  TIM16 timer clock enable
                TIM17EN: u1,
                reserved20: u1,
                ///  ADC clock enable
                ADCEN: u1,
                padding: u11,
            }),
            ///  GPIO in Sleep mode clock enable register
            IOPSMENR: mmio.Mmio(packed struct(u32) {
                ///  I/O port A clock enable during Sleep mode
                IOPASMEN: u1,
                ///  I/O port B clock enable during Sleep mode
                IOPBSMEN: u1,
                ///  I/O port C clock enable during Sleep mode
                IOPCSMEN: u1,
                ///  I/O port D clock enable during Sleep mode
                IOPDSMEN: u1,
                reserved5: u1,
                ///  I/O port F clock enable during Sleep mode
                IOPFSMEN: u1,
                padding: u26,
            }),
            ///  AHB peripheral clock enable in Sleep mode register
            AHBSMENR: mmio.Mmio(packed struct(u32) {
                ///  DMA clock enable during Sleep mode
                DMASMEN: u1,
                reserved8: u7,
                ///  Flash memory interface clock enable during Sleep mode
                FLASHSMEN: u1,
                ///  SRAM clock enable during Sleep mode
                SRAMSMEN: u1,
                reserved12: u2,
                ///  CRC clock enable during Sleep mode
                CRCSMEN: u1,
                padding: u19,
            }),
            ///  APB peripheral clock enable in Sleep mode register 1
            APBSMENR1: mmio.Mmio(packed struct(u32) {
                ///  TIM2 timer clock enable during Sleep mode
                TIM2SMEN: u1,
                ///  TIM3 timer clock enable during Sleep mode
                TIM3SMEN: u1,
                reserved10: u8,
                ///  RTC APB clock enable during Sleep mode
                RTCAPBSMEN: u1,
                ///  WWDG clock enable during Sleep mode
                WWDGSMEN: u1,
                reserved14: u2,
                ///  SPI2 clock enable during Sleep mode
                SPI2SMEN: u1,
                reserved17: u2,
                ///  USART2 clock enable during Sleep mode
                USART2SMEN: u1,
                reserved21: u3,
                ///  I2C1 clock enable during Sleep mode
                I2C1SMEN: u1,
                ///  I2C2 clock enable during Sleep mode
                I2C2SMEN: u1,
                reserved27: u4,
                ///  Debug support clock enable during Sleep mode
                DBGSMEN: u1,
                ///  Power interface clock enable during Sleep mode
                PWRSMEN: u1,
                padding: u3,
            }),
            ///  APB peripheral clock enable in Sleep mode register 2
            APBSMENR2: mmio.Mmio(packed struct(u32) {
                ///  SYSCFG, COMP and VREFBUF clock enable during Sleep mode
                SYSCFGSMEN: u1,
                reserved11: u10,
                ///  TIM1 timer clock enable during Sleep mode
                TIM1SMEN: u1,
                ///  SPI1 clock enable during Sleep mode
                SPI1SMEN: u1,
                reserved14: u1,
                ///  USART1 clock enable during Sleep mode
                USART1SMEN: u1,
                ///  TIM14 timer clock enable during Sleep mode
                TIM14SMEN: u1,
                reserved17: u1,
                ///  TIM16 timer clock enable during Sleep mode
                TIM16SMEN: u1,
                ///  TIM16 timer clock enable during Sleep mode
                TIM17SMEN: u1,
                reserved20: u1,
                ///  ADC clock enable during Sleep mode
                ADCSMEN: u1,
                padding: u11,
            }),
            ///  Peripherals independent clock configuration register
            CCIPR: mmio.Mmio(packed struct(u32) {
                ///  USART1 clock source selection
                USART1SEL: u2,
                reserved12: u10,
                ///  I2C1 clock source selection
                I2C1SEL: u2,
                ///  I2S1 clock source selection
                I2S2SEL: u2,
                reserved22: u6,
                ///  TIM1 clock source selection
                TIM1SEL: u1,
                reserved26: u3,
                ///  RNG clock source selection
                RNGSEL: u2,
                ///  Division factor of RNG clock divider
                RNGDIV: u2,
                ///  ADCs clock source selection
                ADCSEL: u2,
            }),
            reserved92: [4]u8,
            ///  RTC domain control register
            BDCR: mmio.Mmio(packed struct(u32) {
                ///  LSE oscillator enable
                LSEON: u1,
                ///  LSE oscillator ready
                LSERDY: u1,
                ///  LSE oscillator bypass
                LSEBYP: u1,
                ///  LSE oscillator drive capability
                LSEDRV: u2,
                ///  CSS on LSE enable
                LSECSSON: u1,
                ///  CSS on LSE failure Detection
                LSECSSD: u1,
                reserved8: u1,
                ///  RTC clock source selection
                RTCSEL: u2,
                reserved15: u5,
                ///  RTC clock enable
                RTCEN: u1,
                ///  RTC domain software reset
                BDRST: u1,
                reserved24: u7,
                ///  Low-speed clock output (LSCO) enable
                LSCOEN: u1,
                ///  Low-speed clock output selection
                LSCOSEL: u1,
                padding: u6,
            }),
            ///  Control/status register
            CSR: mmio.Mmio(packed struct(u32) {
                ///  LSI oscillator enable
                LSION: u1,
                ///  LSI oscillator ready
                LSIRDY: u1,
                reserved23: u21,
                ///  Remove reset flags
                RMVF: u1,
                reserved25: u1,
                ///  Option byte loader reset flag
                OBLRSTF: u1,
                ///  Pin reset flag
                PINRSTF: u1,
                ///  BOR or POR/PDR flag
                PWRRSTF: u1,
                ///  Software reset flag
                SFTRSTF: u1,
                ///  Independent window watchdog reset flag
                IWDGRSTF: u1,
                ///  Window watchdog reset flag
                WWDGRSTF: u1,
                ///  Low-power reset flag
                LPWRRSTF: u1,
            }),
        };

        ///  Power control
        pub const PWR = extern struct {
            ///  Power control register 1
            CR1: mmio.Mmio(packed struct(u32) {
                ///  Low-power mode selection
                LPMS: u3,
                ///  Flash memory powered down during Stop mode
                FPD_STOP: u1,
                ///  Flash memory powered down during Low-power run mode
                FPD_LPRUN: u1,
                ///  Flash memory powered down during Low-power sleep mode
                FPD_LPSLP: u1,
                reserved8: u2,
                ///  Disable backup domain write protection
                DBP: u1,
                ///  Voltage scaling range selection
                VOS: u2,
                reserved14: u3,
                ///  Low-power run
                LPR: u1,
                padding: u17,
            }),
            ///  Power control register 2
            CR2: mmio.Mmio(packed struct(u32) {
                ///  Power voltage detector enable
                PVDE: u1,
                ///  Power voltage detector falling threshold selection
                PVDFT: u3,
                ///  Power voltage detector rising threshold selection
                PVDRT: u3,
                padding: u25,
            }),
            ///  Power control register 3
            CR3: mmio.Mmio(packed struct(u32) {
                ///  Enable Wakeup pin WKUP1
                EWUP1: u1,
                ///  Enable Wakeup pin WKUP2
                EWUP2: u1,
                reserved3: u1,
                ///  Enable Wakeup pin WKUP4
                EWUP4: u1,
                ///  Enable WKUP5 wakeup pin
                EWUP5: u1,
                ///  Enable WKUP6 wakeup pin
                EWUP6: u1,
                reserved8: u2,
                ///  SRAM retention in Standby mode
                RRS: u1,
                ///  Enable the periodical sampling mode for PDR detection
                ULPEN: u1,
                ///  Apply pull-up and pull-down configuration
                APC: u1,
                reserved15: u4,
                ///  Enable internal wakeup line
                EIWUL: u1,
                padding: u16,
            }),
            ///  Power control register 4
            CR4: mmio.Mmio(packed struct(u32) {
                ///  Wakeup pin WKUP1 polarity
                WP1: u1,
                ///  Wakeup pin WKUP2 polarity
                WP2: u1,
                reserved3: u1,
                ///  Wakeup pin WKUP4 polarity
                WP4: u1,
                ///  Wakeup pin WKUP5 polarity
                WP5: u1,
                ///  WKUP6 wakeup pin polarity
                WP6: u1,
                reserved8: u2,
                ///  VBAT battery charging enable
                VBE: u1,
                ///  VBAT battery charging resistor selection
                VBRS: u1,
                padding: u22,
            }),
            ///  Power status register 1
            SR1: mmio.Mmio(packed struct(u32) {
                ///  Wakeup flag 1
                WUF1: u1,
                ///  Wakeup flag 2
                WUF2: u1,
                reserved3: u1,
                ///  Wakeup flag 4
                WUF4: u1,
                ///  Wakeup flag 5
                WUF5: u1,
                ///  Wakeup flag 6
                WUF6: u1,
                reserved8: u2,
                ///  Standby flag
                SBF: u1,
                reserved15: u6,
                ///  Wakeup flag internal
                WUFI: u1,
                padding: u16,
            }),
            ///  Power status register 2
            SR2: mmio.Mmio(packed struct(u32) {
                reserved7: u7,
                ///  Flash ready flag
                FLASH_RDY: u1,
                ///  Low-power regulator started
                REGLPS: u1,
                ///  Low-power regulator flag
                REGLPF: u1,
                ///  Voltage scaling flag
                VOSF: u1,
                ///  Power voltage detector output
                PVDO: u1,
                padding: u20,
            }),
            ///  Power status clear register
            SCR: mmio.Mmio(packed struct(u32) {
                ///  Clear wakeup flag 1
                CWUF1: u1,
                ///  Clear wakeup flag 2
                CWUF2: u1,
                reserved3: u1,
                ///  Clear wakeup flag 4
                CWUF4: u1,
                ///  Clear wakeup flag 5
                CWUF5: u1,
                ///  Clear wakeup flag 6
                CWUF6: u1,
                reserved8: u2,
                ///  Clear standby flag
                CSBF: u1,
                padding: u23,
            }),
            reserved32: [4]u8,
            ///  Power Port A pull-up control register
            PUCRA: mmio.Mmio(packed struct(u32) {
                ///  Port A pull-up bit y (y=0..15)
                PU0: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU1: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU2: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU3: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU4: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU5: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU6: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU7: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU8: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU9: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU10: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU11: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU12: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU13: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU14: u1,
                ///  Port A pull-up bit y (y=0..15)
                PU15: u1,
                padding: u16,
            }),
            ///  Power Port A pull-down control register
            PDCRA: mmio.Mmio(packed struct(u32) {
                ///  Port A pull-down bit y (y=0..15)
                PD0: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD1: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD2: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD3: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD4: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD5: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD6: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD7: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD8: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD9: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD10: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD11: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD12: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD13: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD14: u1,
                ///  Port A pull-down bit y (y=0..15)
                PD15: u1,
                padding: u16,
            }),
            ///  Power Port B pull-up control register
            PUCRB: mmio.Mmio(packed struct(u32) {
                ///  Port B pull-up bit y (y=0..15)
                PU0: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU1: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU2: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU3: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU4: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU5: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU6: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU7: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU8: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU9: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU10: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU11: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU12: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU13: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU14: u1,
                ///  Port B pull-up bit y (y=0..15)
                PU15: u1,
                padding: u16,
            }),
            ///  Power Port B pull-down control register
            PDCRB: mmio.Mmio(packed struct(u32) {
                ///  Port B pull-down bit y (y=0..15)
                PD0: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD1: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD2: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD3: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD4: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD5: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD6: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD7: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD8: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD9: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD10: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD11: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD12: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD13: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD14: u1,
                ///  Port B pull-down bit y (y=0..15)
                PD15: u1,
                padding: u16,
            }),
            ///  Power Port C pull-up control register
            PUCRC: mmio.Mmio(packed struct(u32) {
                reserved6: u6,
                ///  Port C pull-up bit y (y=0..15)
                PU6: u1,
                ///  Port C pull-up bit y (y=0..15)
                PU7: u1,
                reserved13: u5,
                ///  Port C pull-up bit y (y=0..15)
                PU13: u1,
                ///  Port C pull-up bit y (y=0..15)
                PU14: u1,
                ///  Port C pull-up bit y (y=0..15)
                PU15: u1,
                padding: u16,
            }),
            ///  Power Port C pull-down control register
            PDCRC: mmio.Mmio(packed struct(u32) {
                ///  Port C pull-down bit y (y=0..15)
                PD0: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD1: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD2: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD3: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD4: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD5: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD6: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD7: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD8: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD9: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD10: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD11: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD12: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD13: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD14: u1,
                ///  Port C pull-down bit y (y=0..15)
                PD15: u1,
                padding: u16,
            }),
            ///  Power Port D pull-up control register
            PUCRD: mmio.Mmio(packed struct(u32) {
                ///  Port D pull-up bit y (y=0..15)
                PU0: u1,
                ///  Port D pull-up bit y (y=0..15)
                PU1: u1,
                ///  Port D pull-up bit y (y=0..15)
                PU2: u1,
                ///  Port D pull-up bit y (y=0..15)
                PU3: u1,
                padding: u28,
            }),
            ///  Power Port D pull-down control register
            PDCRD: mmio.Mmio(packed struct(u32) {
                ///  Port D pull-down bit y (y=0..15)
                PD0: u1,
                ///  Port D pull-down bit y (y=0..15)
                PD1: u1,
                ///  Port D pull-down bit y (y=0..15)
                PD2: u1,
                ///  Port D pull-down bit y (y=0..15)
                PD3: u1,
                ///  Port D pull-down bit y (y=0..15)
                PD4: u1,
                ///  Port D pull-down bit y (y=0..15)
                PD5: u1,
                ///  Port D pull-down bit y (y=0..15)
                PD6: u1,
                reserved8: u1,
                ///  Port D pull-down bit y (y=0..15)
                PD8: u1,
                ///  Port D pull-down bit y (y=0..15)
                PD9: u1,
                padding: u22,
            }),
            reserved72: [8]u8,
            ///  Power Port F pull-up control register
            PUCRF: mmio.Mmio(packed struct(u32) {
                ///  Port F pull-up bit y (y=0..15)
                PU0: u1,
                ///  Port F pull-up bit y (y=0..15)
                PU1: u1,
                ///  Port F pull-up bit y (y=0..15)
                PU2: u1,
                padding: u29,
            }),
            ///  Power Port F pull-down control register
            PDCRF: mmio.Mmio(packed struct(u32) {
                ///  Port F pull-down bit y (y=0..15)
                PD0: u1,
                ///  Port F pull-down bit y (y=0..15)
                PD1: u1,
                ///  Port F pull-down bit y (y=0..15)
                PD2: u1,
                padding: u29,
            }),
        };

        ///  DMA controller
        pub const DMA = extern struct {
            ///  low interrupt status register
            ISR: mmio.Mmio(packed struct(u32) {
                ///  Channel global interrupt flag
                GIF0: u1,
                ///  Channel transfer complete flag
                TCIF1: u1,
                ///  Channel half transfer flag
                HTIF2: u1,
                ///  Channel transfer error flag
                TEIF3: u1,
                ///  Channel global interrupt flag
                GIF4: u1,
                ///  Channel transfer complete flag
                TCIF5: u1,
                ///  Channel half transfer flag
                HTIF6: u1,
                ///  Channel transfer error flag
                TEIF7: u1,
                ///  Channel global interrupt flag
                GIF8: u1,
                ///  Channel transfer complete flag
                TCIF9: u1,
                ///  Channel half transfer flag
                HTIF10: u1,
                ///  Channel transfer error flag
                TEIF11: u1,
                ///  Channel global interrupt flag
                GIF12: u1,
                ///  Channel transfer complete flag
                TCIF13: u1,
                ///  Channel half transfer flag
                HTIF14: u1,
                ///  Channel transfer error flag
                TEIF15: u1,
                ///  Channel global interrupt flag
                GIF16: u1,
                ///  Channel transfer complete flag
                TCIF17: u1,
                ///  Channel half transfer flag
                HTIF18: u1,
                ///  Channel transfer error flag
                TEIF19: u1,
                ///  Channel global interrupt flag
                GIF20: u1,
                ///  Channel transfer complete flag
                TCIF21: u1,
                ///  Channel half transfer flag
                HTIF22: u1,
                ///  Channel transfer error flag
                TEIF23: u1,
                ///  Channel global interrupt flag
                GIF24: u1,
                ///  Channel transfer complete flag
                TCIF25: u1,
                ///  Channel half transfer flag
                HTIF26: u1,
                ///  Channel transfer error flag
                TEIF27: u1,
                padding: u4,
            }),
            ///  high interrupt status register
            IFCR: mmio.Mmio(packed struct(u32) {
                ///  Channel global interrupt flag
                CGIF0: u1,
                ///  Channel transfer complete flag
                CTCIF1: u1,
                ///  Channel half transfer flag
                CHTIF2: u1,
                ///  Channel transfer error flag
                CTEIF3: u1,
                ///  Channel global interrupt flag
                CGIF4: u1,
                ///  Channel transfer complete flag
                CTCIF5: u1,
                ///  Channel half transfer flag
                CHTIF6: u1,
                ///  Channel transfer error flag
                CTEIF7: u1,
                ///  Channel global interrupt flag
                CGIF8: u1,
                ///  Channel transfer complete flag
                CTCIF9: u1,
                ///  Channel half transfer flag
                CHTIF10: u1,
                ///  Channel transfer error flag
                CTEIF11: u1,
                ///  Channel global interrupt flag
                CGIF12: u1,
                ///  Channel transfer complete flag
                CTCIF13: u1,
                ///  Channel half transfer flag
                CHTIF14: u1,
                ///  Channel transfer error flag
                CTEIF15: u1,
                ///  Channel global interrupt flag
                CGIF16: u1,
                ///  Channel transfer complete flag
                CTCIF17: u1,
                ///  Channel half transfer flag
                CHTIF18: u1,
                ///  Channel transfer error flag
                CTEIF19: u1,
                ///  Channel global interrupt flag
                CGIF20: u1,
                ///  Channel transfer complete flag
                CTCIF21: u1,
                ///  Channel half transfer flag
                CHTIF22: u1,
                ///  Channel transfer error flag
                CTEIF23: u1,
                ///  Channel global interrupt flag
                CGIF24: u1,
                ///  Channel transfer complete flag
                CTCIF25: u1,
                ///  Channel half transfer flag
                CHTIF26: u1,
                ///  Channel transfer error flag
                CTEIF27: u1,
                padding: u4,
            }),
            ///  DMA channel x configuration register
            CCR1: mmio.Mmio(packed struct(u32) {
                ///  Channel enable
                EN: u1,
                ///  Transfer complete interrupt enable
                TCIE: u1,
                ///  Half transfer interrupt enable
                HTIE: u1,
                ///  Transfer error interrupt enable
                TEIE: u1,
                ///  Data transfer direction
                DIR: u1,
                ///  Circular mode
                CIRC: u1,
                ///  Peripheral increment mode
                PINC: u1,
                ///  Memory increment mode
                MINC: u1,
                ///  Peripheral size
                PSIZE: u2,
                ///  Memory size
                MSIZE: u2,
                ///  Channel priority level
                PL: u2,
                ///  Memory to memory mode
                MEM2MEM: u1,
                padding: u17,
            }),
            ///  DMA channel x number of data register
            CNDTR1: mmio.Mmio(packed struct(u32) {
                ///  Number of data to transfer
                NDT: u16,
                padding: u16,
            }),
            ///  DMA channel x peripheral address register
            CPAR1: mmio.Mmio(packed struct(u32) {
                ///  Peripheral address
                PA: u32,
            }),
            ///  DMA channel x memory address register
            CMAR1: mmio.Mmio(packed struct(u32) {
                ///  Memory address
                MA: u32,
            }),
            reserved28: [4]u8,
            ///  DMA channel x configuration register
            CCR2: mmio.Mmio(packed struct(u32) {
                ///  Channel enable
                EN: u1,
                ///  Transfer complete interrupt enable
                TCIE: u1,
                ///  Half transfer interrupt enable
                HTIE: u1,
                ///  Transfer error interrupt enable
                TEIE: u1,
                ///  Data transfer direction
                DIR: u1,
                ///  Circular mode
                CIRC: u1,
                ///  Peripheral increment mode
                PINC: u1,
                ///  Memory increment mode
                MINC: u1,
                ///  Peripheral size
                PSIZE: u2,
                ///  Memory size
                MSIZE: u2,
                ///  Channel priority level
                PL: u2,
                ///  Memory to memory mode
                MEM2MEM: u1,
                padding: u17,
            }),
            ///  DMA channel x number of data register
            CNDTR2: mmio.Mmio(packed struct(u32) {
                ///  Number of data to transfer
                NDT: u16,
                padding: u16,
            }),
            ///  DMA channel x peripheral address register
            CPAR2: mmio.Mmio(packed struct(u32) {
                ///  Peripheral address
                PA: u32,
            }),
            ///  DMA channel x memory address register
            CMAR2: mmio.Mmio(packed struct(u32) {
                ///  Memory address
                MA: u32,
            }),
            reserved48: [4]u8,
            ///  DMA channel x configuration register
            CCR3: mmio.Mmio(packed struct(u32) {
                ///  Channel enable
                EN: u1,
                ///  Transfer complete interrupt enable
                TCIE: u1,
                ///  Half transfer interrupt enable
                HTIE: u1,
                ///  Transfer error interrupt enable
                TEIE: u1,
                ///  Data transfer direction
                DIR: u1,
                ///  Circular mode
                CIRC: u1,
                ///  Peripheral increment mode
                PINC: u1,
                ///  Memory increment mode
                MINC: u1,
                ///  Peripheral size
                PSIZE: u2,
                ///  Memory size
                MSIZE: u2,
                ///  Channel priority level
                PL: u2,
                ///  Memory to memory mode
                MEM2MEM: u1,
                padding: u17,
            }),
            ///  DMA channel x configuration register
            CNDTR3: mmio.Mmio(packed struct(u32) {
                ///  Number of data to transfer
                NDT: u16,
                padding: u16,
            }),
            ///  DMA channel x peripheral address register
            CPAR3: mmio.Mmio(packed struct(u32) {
                ///  Peripheral address
                PA: u32,
            }),
            ///  DMA channel x memory address register
            CMAR3: mmio.Mmio(packed struct(u32) {
                ///  Memory address
                MA: u32,
            }),
            reserved68: [4]u8,
            ///  DMA channel x configuration register
            CCR4: mmio.Mmio(packed struct(u32) {
                ///  Channel enable
                EN: u1,
                ///  Transfer complete interrupt enable
                TCIE: u1,
                ///  Half transfer interrupt enable
                HTIE: u1,
                ///  Transfer error interrupt enable
                TEIE: u1,
                ///  Data transfer direction
                DIR: u1,
                ///  Circular mode
                CIRC: u1,
                ///  Peripheral increment mode
                PINC: u1,
                ///  Memory increment mode
                MINC: u1,
                ///  Peripheral size
                PSIZE: u2,
                ///  Memory size
                MSIZE: u2,
                ///  Channel priority level
                PL: u2,
                ///  Memory to memory mode
                MEM2MEM: u1,
                padding: u17,
            }),
            ///  DMA channel x configuration register
            CNDTR4: mmio.Mmio(packed struct(u32) {
                ///  Number of data to transfer
                NDT: u16,
                padding: u16,
            }),
            ///  DMA channel x peripheral address register
            CPAR4: mmio.Mmio(packed struct(u32) {
                ///  Peripheral address
                PA: u32,
            }),
            ///  DMA channel x memory address register
            CMAR4: mmio.Mmio(packed struct(u32) {
                ///  Memory address
                MA: u32,
            }),
            reserved88: [4]u8,
            ///  DMA channel x configuration register
            CCR5: mmio.Mmio(packed struct(u32) {
                ///  Channel enable
                EN: u1,
                ///  Transfer complete interrupt enable
                TCIE: u1,
                ///  Half transfer interrupt enable
                HTIE: u1,
                ///  Transfer error interrupt enable
                TEIE: u1,
                ///  Data transfer direction
                DIR: u1,
                ///  Circular mode
                CIRC: u1,
                ///  Peripheral increment mode
                PINC: u1,
                ///  Memory increment mode
                MINC: u1,
                ///  Peripheral size
                PSIZE: u2,
                ///  Memory size
                MSIZE: u2,
                ///  Channel priority level
                PL: u2,
                ///  Memory to memory mode
                MEM2MEM: u1,
                padding: u17,
            }),
            ///  DMA channel x configuration register
            CNDTR5: mmio.Mmio(packed struct(u32) {
                ///  Number of data to transfer
                NDT: u16,
                padding: u16,
            }),
            ///  DMA channel x peripheral address register
            CPAR5: mmio.Mmio(packed struct(u32) {
                ///  Peripheral address
                PA: u32,
            }),
            ///  DMA channel x memory address register
            CMAR5: mmio.Mmio(packed struct(u32) {
                ///  Memory address
                MA: u32,
            }),
        };

        ///  DMAMUX
        pub const DMAMUX = extern struct {
            ///  DMAMux - DMA request line multiplexer channel x control register
            C0CR: mmio.Mmio(packed struct(u32) {
                ///  Input DMA request line selected
                DMAREQ_ID: u8,
                ///  Interrupt enable at synchronization event overrun
                SOIE: u1,
                ///  Event generation enable/disable
                EGE: u1,
                reserved16: u6,
                ///  Synchronous operating mode enable/disable
                SE: u1,
                ///  Synchronization event type selector Defines the synchronization event on the selected synchronization input:
                SPOL: u2,
                ///  Number of DMA requests to forward Defines the number of DMA requests forwarded before output event is generated. In synchronous mode, it also defines the number of DMA requests to forward after a synchronization event, then stop forwarding. The actual number of DMA requests forwarded is NBREQ+1. Note: This field can only be written when both SE and EGE bits are reset.
                NBREQ: u5,
                ///  Synchronization input selected
                SYNC_ID: u5,
                padding: u3,
            }),
            ///  DMAMux - DMA request line multiplexer channel x control register
            C1CR: mmio.Mmio(packed struct(u32) {
                ///  Input DMA request line selected
                DMAREQ_ID: u8,
                ///  Interrupt enable at synchronization event overrun
                SOIE: u1,
                ///  Event generation enable/disable
                EGE: u1,
                reserved16: u6,
                ///  Synchronous operating mode enable/disable
                SE: u1,
                ///  Synchronization event type selector Defines the synchronization event on the selected synchronization input:
                SPOL: u2,
                ///  Number of DMA requests to forward Defines the number of DMA requests forwarded before output event is generated. In synchronous mode, it also defines the number of DMA requests to forward after a synchronization event, then stop forwarding. The actual number of DMA requests forwarded is NBREQ+1. Note: This field can only be written when both SE and EGE bits are reset.
                NBREQ: u5,
                ///  Synchronization input selected
                SYNC_ID: u5,
                padding: u3,
            }),
            ///  DMAMux - DMA request line multiplexer channel x control register
            C2CR: mmio.Mmio(packed struct(u32) {
                ///  Input DMA request line selected
                DMAREQ_ID: u8,
                ///  Interrupt enable at synchronization event overrun
                SOIE: u1,
                ///  Event generation enable/disable
                EGE: u1,
                reserved16: u6,
                ///  Synchronous operating mode enable/disable
                SE: u1,
                ///  Synchronization event type selector Defines the synchronization event on the selected synchronization input:
                SPOL: u2,
                ///  Number of DMA requests to forward Defines the number of DMA requests forwarded before output event is generated. In synchronous mode, it also defines the number of DMA requests to forward after a synchronization event, then stop forwarding. The actual number of DMA requests forwarded is NBREQ+1. Note: This field can only be written when both SE and EGE bits are reset.
                NBREQ: u5,
                ///  Synchronization input selected
                SYNC_ID: u5,
                padding: u3,
            }),
            ///  DMAMux - DMA request line multiplexer channel x control register
            C3CR: mmio.Mmio(packed struct(u32) {
                ///  Input DMA request line selected
                DMAREQ_ID: u8,
                ///  Interrupt enable at synchronization event overrun
                SOIE: u1,
                ///  Event generation enable/disable
                EGE: u1,
                reserved16: u6,
                ///  Synchronous operating mode enable/disable
                SE: u1,
                ///  Synchronization event type selector Defines the synchronization event on the selected synchronization input:
                SPOL: u2,
                ///  Number of DMA requests to forward Defines the number of DMA requests forwarded before output event is generated. In synchronous mode, it also defines the number of DMA requests to forward after a synchronization event, then stop forwarding. The actual number of DMA requests forwarded is NBREQ+1. Note: This field can only be written when both SE and EGE bits are reset.
                NBREQ: u5,
                ///  Synchronization input selected
                SYNC_ID: u5,
                padding: u3,
            }),
            ///  DMAMux - DMA request line multiplexer channel x control register
            C4CR: mmio.Mmio(packed struct(u32) {
                ///  Input DMA request line selected
                DMAREQ_ID: u8,
                ///  Interrupt enable at synchronization event overrun
                SOIE: u1,
                ///  Event generation enable/disable
                EGE: u1,
                reserved16: u6,
                ///  Synchronous operating mode enable/disable
                SE: u1,
                ///  Synchronization event type selector Defines the synchronization event on the selected synchronization input:
                SPOL: u2,
                ///  Number of DMA requests to forward Defines the number of DMA requests forwarded before output event is generated. In synchronous mode, it also defines the number of DMA requests to forward after a synchronization event, then stop forwarding. The actual number of DMA requests forwarded is NBREQ+1. Note: This field can only be written when both SE and EGE bits are reset.
                NBREQ: u5,
                ///  Synchronization input selected
                SYNC_ID: u5,
                padding: u3,
            }),
            ///  DMAMux - DMA request line multiplexer channel x control register
            C5CR: mmio.Mmio(packed struct(u32) {
                ///  Input DMA request line selected
                DMAREQ_ID: u8,
                ///  Interrupt enable at synchronization event overrun
                SOIE: u1,
                ///  Event generation enable/disable
                EGE: u1,
                reserved16: u6,
                ///  Synchronous operating mode enable/disable
                SE: u1,
                ///  Synchronization event type selector Defines the synchronization event on the selected synchronization input:
                SPOL: u2,
                ///  Number of DMA requests to forward Defines the number of DMA requests forwarded before output event is generated. In synchronous mode, it also defines the number of DMA requests to forward after a synchronization event, then stop forwarding. The actual number of DMA requests forwarded is NBREQ+1. Note: This field can only be written when both SE and EGE bits are reset.
                NBREQ: u5,
                ///  Synchronization input selected
                SYNC_ID: u5,
                padding: u3,
            }),
            ///  DMAMux - DMA request line multiplexer channel x control register
            C6CR: mmio.Mmio(packed struct(u32) {
                ///  Input DMA request line selected
                DMAREQ_ID: u8,
                ///  Interrupt enable at synchronization event overrun
                SOIE: u1,
                ///  Event generation enable/disable
                EGE: u1,
                reserved16: u6,
                ///  Synchronous operating mode enable/disable
                SE: u1,
                ///  Synchronization event type selector Defines the synchronization event on the selected synchronization input:
                SPOL: u2,
                ///  Number of DMA requests to forward Defines the number of DMA requests forwarded before output event is generated. In synchronous mode, it also defines the number of DMA requests to forward after a synchronization event, then stop forwarding. The actual number of DMA requests forwarded is NBREQ+1. Note: This field can only be written when both SE and EGE bits are reset.
                NBREQ: u5,
                ///  Synchronization input selected
                SYNC_ID: u5,
                padding: u3,
            }),
            reserved256: [228]u8,
            ///  DMAMux - DMA request generator channel x control register
            RG0CR: mmio.Mmio(packed struct(u32) {
                ///  DMA request trigger input selected
                SIG_ID: u5,
                reserved8: u3,
                ///  Interrupt enable at trigger event overrun
                OIE: u1,
                reserved16: u7,
                ///  DMA request generator channel enable/disable
                GE: u1,
                ///  DMA request generator trigger event type selection Defines the trigger event on the selected DMA request trigger input
                GPOL: u2,
                ///  Number of DMA requests to generate Defines the number of DMA requests generated after a trigger event, then stop generating. The actual number of generated DMA requests is GNBREQ+1. Note: This field can only be written when GE bit is reset.
                GNBREQ: u5,
                padding: u8,
            }),
            ///  DMAMux - DMA request generator channel x control register
            RG1CR: mmio.Mmio(packed struct(u32) {
                ///  DMA request trigger input selected
                SIG_ID: u5,
                reserved8: u3,
                ///  Interrupt enable at trigger event overrun
                OIE: u1,
                reserved16: u7,
                ///  DMA request generator channel enable/disable
                GE: u1,
                ///  DMA request generator trigger event type selection Defines the trigger event on the selected DMA request trigger input
                GPOL: u2,
                ///  Number of DMA requests to generate Defines the number of DMA requests generated after a trigger event, then stop generating. The actual number of generated DMA requests is GNBREQ+1. Note: This field can only be written when GE bit is reset.
                GNBREQ: u5,
                padding: u8,
            }),
            ///  DMAMux - DMA request generator channel x control register
            RG2CR: mmio.Mmio(packed struct(u32) {
                ///  DMA request trigger input selected
                SIG_ID: u5,
                reserved8: u3,
                ///  Interrupt enable at trigger event overrun
                OIE: u1,
                reserved16: u7,
                ///  DMA request generator channel enable/disable
                GE: u1,
                ///  DMA request generator trigger event type selection Defines the trigger event on the selected DMA request trigger input
                GPOL: u2,
                ///  Number of DMA requests to generate Defines the number of DMA requests generated after a trigger event, then stop generating. The actual number of generated DMA requests is GNBREQ+1. Note: This field can only be written when GE bit is reset.
                GNBREQ: u5,
                padding: u8,
            }),
            ///  DMAMux - DMA request generator channel x control register
            RG3CR: mmio.Mmio(packed struct(u32) {
                ///  DMA request trigger input selected
                SIG_ID: u5,
                reserved8: u3,
                ///  Interrupt enable at trigger event overrun
                OIE: u1,
                reserved16: u7,
                ///  DMA request generator channel enable/disable
                GE: u1,
                ///  DMA request generator trigger event type selection Defines the trigger event on the selected DMA request trigger input
                GPOL: u2,
                ///  Number of DMA requests to generate Defines the number of DMA requests generated after a trigger event, then stop generating. The actual number of generated DMA requests is GNBREQ+1. Note: This field can only be written when GE bit is reset.
                GNBREQ: u5,
                padding: u8,
            }),
            reserved320: [48]u8,
            ///  DMAMux - DMA request generator status register
            RGSR: mmio.Mmio(packed struct(u32) {
                ///  Trigger event overrun flag The flag is set when a trigger event occurs on DMA request generator channel x, while the DMA request generator counter value is lower than GNBREQ. The flag is cleared by writing 1 to the corresponding COFx bit in DMAMUX_RGCFR register.
                OF: u4,
                padding: u28,
            }),
            ///  DMAMux - DMA request generator clear flag register
            RGCFR: mmio.Mmio(packed struct(u32) {
                ///  Clear trigger event overrun flag Upon setting, this bit clears the corresponding overrun flag OFx in the DMAMUX_RGCSR register.
                COF: u4,
                padding: u28,
            }),
        };

        ///  General-purpose I/Os
        pub const GPIOA = extern struct {
            ///  GPIO port mode register
            MODER: mmio.Mmio(packed struct(u32) {
                ///  Port x configuration bits (y = 0..15)
                MODER0: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER1: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER2: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER3: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER4: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER5: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER6: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER7: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER8: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER9: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER10: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER11: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER12: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER13: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER14: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER15: u2,
            }),
            ///  GPIO port output type register
            OTYPER: mmio.Mmio(packed struct(u32) {
                ///  Port x configuration bits (y = 0..15)
                OT0: u1,
                ///  Port x configuration bits (y = 0..15)
                OT1: u1,
                ///  Port x configuration bits (y = 0..15)
                OT2: u1,
                ///  Port x configuration bits (y = 0..15)
                OT3: u1,
                ///  Port x configuration bits (y = 0..15)
                OT4: u1,
                ///  Port x configuration bits (y = 0..15)
                OT5: u1,
                ///  Port x configuration bits (y = 0..15)
                OT6: u1,
                ///  Port x configuration bits (y = 0..15)
                OT7: u1,
                ///  Port x configuration bits (y = 0..15)
                OT8: u1,
                ///  Port x configuration bits (y = 0..15)
                OT9: u1,
                ///  Port x configuration bits (y = 0..15)
                OT10: u1,
                ///  Port x configuration bits (y = 0..15)
                OT11: u1,
                ///  Port x configuration bits (y = 0..15)
                OT12: u1,
                ///  Port x configuration bits (y = 0..15)
                OT13: u1,
                ///  Port x configuration bits (y = 0..15)
                OT14: u1,
                ///  Port x configuration bits (y = 0..15)
                OT15: u1,
                padding: u16,
            }),
            ///  GPIO port output speed register
            OSPEEDR: mmio.Mmio(packed struct(u32) {
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR0: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR1: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR2: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR3: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR4: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR5: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR6: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR7: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR8: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR9: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR10: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR11: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR12: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR13: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR14: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR15: u2,
            }),
            ///  GPIO port pull-up/pull-down register
            PUPDR: mmio.Mmio(packed struct(u32) {
                ///  Port x configuration bits (y = 0..15)
                PUPDR0: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR1: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR2: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR3: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR4: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR5: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR6: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR7: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR8: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR9: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR10: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR11: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR12: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR13: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR14: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR15: u2,
            }),
            ///  GPIO port input data register
            IDR: mmio.Mmio(packed struct(u32) {
                ///  Port input data (y = 0..15)
                IDR0: u1,
                ///  Port input data (y = 0..15)
                IDR1: u1,
                ///  Port input data (y = 0..15)
                IDR2: u1,
                ///  Port input data (y = 0..15)
                IDR3: u1,
                ///  Port input data (y = 0..15)
                IDR4: u1,
                ///  Port input data (y = 0..15)
                IDR5: u1,
                ///  Port input data (y = 0..15)
                IDR6: u1,
                ///  Port input data (y = 0..15)
                IDR7: u1,
                ///  Port input data (y = 0..15)
                IDR8: u1,
                ///  Port input data (y = 0..15)
                IDR9: u1,
                ///  Port input data (y = 0..15)
                IDR10: u1,
                ///  Port input data (y = 0..15)
                IDR11: u1,
                ///  Port input data (y = 0..15)
                IDR12: u1,
                ///  Port input data (y = 0..15)
                IDR13: u1,
                ///  Port input data (y = 0..15)
                IDR14: u1,
                ///  Port input data (y = 0..15)
                IDR15: u1,
                padding: u16,
            }),
            ///  GPIO port output data register
            ODR: mmio.Mmio(packed struct(u32) {
                ///  Port output data (y = 0..15)
                ODR0: u1,
                ///  Port output data (y = 0..15)
                ODR1: u1,
                ///  Port output data (y = 0..15)
                ODR2: u1,
                ///  Port output data (y = 0..15)
                ODR3: u1,
                ///  Port output data (y = 0..15)
                ODR4: u1,
                ///  Port output data (y = 0..15)
                ODR5: u1,
                ///  Port output data (y = 0..15)
                ODR6: u1,
                ///  Port output data (y = 0..15)
                ODR7: u1,
                ///  Port output data (y = 0..15)
                ODR8: u1,
                ///  Port output data (y = 0..15)
                ODR9: u1,
                ///  Port output data (y = 0..15)
                ODR10: u1,
                ///  Port output data (y = 0..15)
                ODR11: u1,
                ///  Port output data (y = 0..15)
                ODR12: u1,
                ///  Port output data (y = 0..15)
                ODR13: u1,
                ///  Port output data (y = 0..15)
                ODR14: u1,
                ///  Port output data (y = 0..15)
                ODR15: u1,
                padding: u16,
            }),
            ///  GPIO port bit set/reset register
            BSRR: mmio.Mmio(packed struct(u32) {
                ///  Port x set bit y (y= 0..15)
                BS0: u1,
                ///  Port x set bit y (y= 0..15)
                BS1: u1,
                ///  Port x set bit y (y= 0..15)
                BS2: u1,
                ///  Port x set bit y (y= 0..15)
                BS3: u1,
                ///  Port x set bit y (y= 0..15)
                BS4: u1,
                ///  Port x set bit y (y= 0..15)
                BS5: u1,
                ///  Port x set bit y (y= 0..15)
                BS6: u1,
                ///  Port x set bit y (y= 0..15)
                BS7: u1,
                ///  Port x set bit y (y= 0..15)
                BS8: u1,
                ///  Port x set bit y (y= 0..15)
                BS9: u1,
                ///  Port x set bit y (y= 0..15)
                BS10: u1,
                ///  Port x set bit y (y= 0..15)
                BS11: u1,
                ///  Port x set bit y (y= 0..15)
                BS12: u1,
                ///  Port x set bit y (y= 0..15)
                BS13: u1,
                ///  Port x set bit y (y= 0..15)
                BS14: u1,
                ///  Port x set bit y (y= 0..15)
                BS15: u1,
                ///  Port x set bit y (y= 0..15)
                BR0: u1,
                ///  Port x reset bit y (y = 0..15)
                BR1: u1,
                ///  Port x reset bit y (y = 0..15)
                BR2: u1,
                ///  Port x reset bit y (y = 0..15)
                BR3: u1,
                ///  Port x reset bit y (y = 0..15)
                BR4: u1,
                ///  Port x reset bit y (y = 0..15)
                BR5: u1,
                ///  Port x reset bit y (y = 0..15)
                BR6: u1,
                ///  Port x reset bit y (y = 0..15)
                BR7: u1,
                ///  Port x reset bit y (y = 0..15)
                BR8: u1,
                ///  Port x reset bit y (y = 0..15)
                BR9: u1,
                ///  Port x reset bit y (y = 0..15)
                BR10: u1,
                ///  Port x reset bit y (y = 0..15)
                BR11: u1,
                ///  Port x reset bit y (y = 0..15)
                BR12: u1,
                ///  Port x reset bit y (y = 0..15)
                BR13: u1,
                ///  Port x reset bit y (y = 0..15)
                BR14: u1,
                ///  Port x reset bit y (y = 0..15)
                BR15: u1,
            }),
            ///  GPIO port configuration lock register
            LCKR: mmio.Mmio(packed struct(u32) {
                ///  Port x lock bit y (y= 0..15)
                LCK0: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK1: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK2: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK3: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK4: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK5: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK6: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK7: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK8: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK9: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK10: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK11: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK12: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK13: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK14: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK15: u1,
                ///  Port x lock bit y (y= 0..15)
                LCKK: u1,
                padding: u15,
            }),
            ///  GPIO alternate function low register
            AFRL: mmio.Mmio(packed struct(u32) {
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL0: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL1: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL2: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL3: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL4: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL5: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL6: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL7: u4,
            }),
            ///  GPIO alternate function high register
            AFRH: mmio.Mmio(packed struct(u32) {
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL8: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL9: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL10: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL11: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL12: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL13: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL14: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL15: u4,
            }),
            ///  port bit reset register
            BRR: mmio.Mmio(packed struct(u32) {
                ///  Port Reset bit
                BR0: u1,
                ///  Port Reset bit
                BR1: u1,
                ///  Port Reset bit
                BR2: u1,
                ///  Port Reset bit
                BR3: u1,
                ///  Port Reset bit
                BR4: u1,
                ///  Port Reset bit
                BR5: u1,
                ///  Port Reset bit
                BR6: u1,
                ///  Port Reset bit
                BR7: u1,
                ///  Port Reset bit
                BR8: u1,
                ///  Port Reset bit
                BR9: u1,
                ///  Port Reset bit
                BR10: u1,
                ///  Port Reset bit
                BR11: u1,
                ///  Port Reset bit
                BR12: u1,
                ///  Port Reset bit
                BR13: u1,
                ///  Port Reset bit
                BR14: u1,
                ///  Port Reset bit
                BR15: u1,
                padding: u16,
            }),
        };

        ///  General-purpose I/Os
        pub const GPIOB = extern struct {
            ///  GPIO port mode register
            MODER: mmio.Mmio(packed struct(u32) {
                ///  Port x configuration bits (y = 0..15)
                MODER0: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER1: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER2: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER3: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER4: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER5: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER6: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER7: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER8: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER9: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER10: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER11: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER12: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER13: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER14: u2,
                ///  Port x configuration bits (y = 0..15)
                MODER15: u2,
            }),
            ///  GPIO port output type register
            OTYPER: mmio.Mmio(packed struct(u32) {
                ///  Port x configuration bits (y = 0..15)
                OT0: u1,
                ///  Port x configuration bits (y = 0..15)
                OT1: u1,
                ///  Port x configuration bits (y = 0..15)
                OT2: u1,
                ///  Port x configuration bits (y = 0..15)
                OT3: u1,
                ///  Port x configuration bits (y = 0..15)
                OT4: u1,
                ///  Port x configuration bits (y = 0..15)
                OT5: u1,
                ///  Port x configuration bits (y = 0..15)
                OT6: u1,
                ///  Port x configuration bits (y = 0..15)
                OT7: u1,
                ///  Port x configuration bits (y = 0..15)
                OT8: u1,
                ///  Port x configuration bits (y = 0..15)
                OT9: u1,
                ///  Port x configuration bits (y = 0..15)
                OT10: u1,
                ///  Port x configuration bits (y = 0..15)
                OT11: u1,
                ///  Port x configuration bits (y = 0..15)
                OT12: u1,
                ///  Port x configuration bits (y = 0..15)
                OT13: u1,
                ///  Port x configuration bits (y = 0..15)
                OT14: u1,
                ///  Port x configuration bits (y = 0..15)
                OT15: u1,
                padding: u16,
            }),
            ///  GPIO port output speed register
            OSPEEDR: mmio.Mmio(packed struct(u32) {
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR0: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR1: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR2: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR3: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR4: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR5: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR6: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR7: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR8: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR9: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR10: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR11: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR12: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR13: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR14: u2,
                ///  Port x configuration bits (y = 0..15)
                OSPEEDR15: u2,
            }),
            ///  GPIO port pull-up/pull-down register
            PUPDR: mmio.Mmio(packed struct(u32) {
                ///  Port x configuration bits (y = 0..15)
                PUPDR0: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR1: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR2: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR3: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR4: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR5: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR6: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR7: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR8: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR9: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR10: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR11: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR12: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR13: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR14: u2,
                ///  Port x configuration bits (y = 0..15)
                PUPDR15: u2,
            }),
            ///  GPIO port input data register
            IDR: mmio.Mmio(packed struct(u32) {
                ///  Port input data (y = 0..15)
                IDR0: u1,
                ///  Port input data (y = 0..15)
                IDR1: u1,
                ///  Port input data (y = 0..15)
                IDR2: u1,
                ///  Port input data (y = 0..15)
                IDR3: u1,
                ///  Port input data (y = 0..15)
                IDR4: u1,
                ///  Port input data (y = 0..15)
                IDR5: u1,
                ///  Port input data (y = 0..15)
                IDR6: u1,
                ///  Port input data (y = 0..15)
                IDR7: u1,
                ///  Port input data (y = 0..15)
                IDR8: u1,
                ///  Port input data (y = 0..15)
                IDR9: u1,
                ///  Port input data (y = 0..15)
                IDR10: u1,
                ///  Port input data (y = 0..15)
                IDR11: u1,
                ///  Port input data (y = 0..15)
                IDR12: u1,
                ///  Port input data (y = 0..15)
                IDR13: u1,
                ///  Port input data (y = 0..15)
                IDR14: u1,
                ///  Port input data (y = 0..15)
                IDR15: u1,
                padding: u16,
            }),
            ///  GPIO port output data register
            ODR: mmio.Mmio(packed struct(u32) {
                ///  Port output data (y = 0..15)
                ODR0: u1,
                ///  Port output data (y = 0..15)
                ODR1: u1,
                ///  Port output data (y = 0..15)
                ODR2: u1,
                ///  Port output data (y = 0..15)
                ODR3: u1,
                ///  Port output data (y = 0..15)
                ODR4: u1,
                ///  Port output data (y = 0..15)
                ODR5: u1,
                ///  Port output data (y = 0..15)
                ODR6: u1,
                ///  Port output data (y = 0..15)
                ODR7: u1,
                ///  Port output data (y = 0..15)
                ODR8: u1,
                ///  Port output data (y = 0..15)
                ODR9: u1,
                ///  Port output data (y = 0..15)
                ODR10: u1,
                ///  Port output data (y = 0..15)
                ODR11: u1,
                ///  Port output data (y = 0..15)
                ODR12: u1,
                ///  Port output data (y = 0..15)
                ODR13: u1,
                ///  Port output data (y = 0..15)
                ODR14: u1,
                ///  Port output data (y = 0..15)
                ODR15: u1,
                padding: u16,
            }),
            ///  GPIO port bit set/reset register
            BSRR: mmio.Mmio(packed struct(u32) {
                ///  Port x set bit y (y= 0..15)
                BS0: u1,
                ///  Port x set bit y (y= 0..15)
                BS1: u1,
                ///  Port x set bit y (y= 0..15)
                BS2: u1,
                ///  Port x set bit y (y= 0..15)
                BS3: u1,
                ///  Port x set bit y (y= 0..15)
                BS4: u1,
                ///  Port x set bit y (y= 0..15)
                BS5: u1,
                ///  Port x set bit y (y= 0..15)
                BS6: u1,
                ///  Port x set bit y (y= 0..15)
                BS7: u1,
                ///  Port x set bit y (y= 0..15)
                BS8: u1,
                ///  Port x set bit y (y= 0..15)
                BS9: u1,
                ///  Port x set bit y (y= 0..15)
                BS10: u1,
                ///  Port x set bit y (y= 0..15)
                BS11: u1,
                ///  Port x set bit y (y= 0..15)
                BS12: u1,
                ///  Port x set bit y (y= 0..15)
                BS13: u1,
                ///  Port x set bit y (y= 0..15)
                BS14: u1,
                ///  Port x set bit y (y= 0..15)
                BS15: u1,
                ///  Port x set bit y (y= 0..15)
                BR0: u1,
                ///  Port x reset bit y (y = 0..15)
                BR1: u1,
                ///  Port x reset bit y (y = 0..15)
                BR2: u1,
                ///  Port x reset bit y (y = 0..15)
                BR3: u1,
                ///  Port x reset bit y (y = 0..15)
                BR4: u1,
                ///  Port x reset bit y (y = 0..15)
                BR5: u1,
                ///  Port x reset bit y (y = 0..15)
                BR6: u1,
                ///  Port x reset bit y (y = 0..15)
                BR7: u1,
                ///  Port x reset bit y (y = 0..15)
                BR8: u1,
                ///  Port x reset bit y (y = 0..15)
                BR9: u1,
                ///  Port x reset bit y (y = 0..15)
                BR10: u1,
                ///  Port x reset bit y (y = 0..15)
                BR11: u1,
                ///  Port x reset bit y (y = 0..15)
                BR12: u1,
                ///  Port x reset bit y (y = 0..15)
                BR13: u1,
                ///  Port x reset bit y (y = 0..15)
                BR14: u1,
                ///  Port x reset bit y (y = 0..15)
                BR15: u1,
            }),
            ///  GPIO port configuration lock register
            LCKR: mmio.Mmio(packed struct(u32) {
                ///  Port x lock bit y (y= 0..15)
                LCK0: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK1: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK2: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK3: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK4: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK5: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK6: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK7: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK8: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK9: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK10: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK11: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK12: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK13: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK14: u1,
                ///  Port x lock bit y (y= 0..15)
                LCK15: u1,
                ///  Port x lock bit y (y= 0..15)
                LCKK: u1,
                padding: u15,
            }),
            ///  GPIO alternate function low register
            AFRL: mmio.Mmio(packed struct(u32) {
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL0: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL1: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL2: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL3: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL4: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL5: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL6: u4,
                ///  Alternate function selection for port x bit y (y = 0..7)
                AFSEL7: u4,
            }),
            ///  GPIO alternate function high register
            AFRH: mmio.Mmio(packed struct(u32) {
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL8: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL9: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL10: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL11: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL12: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL13: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL14: u4,
                ///  Alternate function selection for port x bit y (y = 8..15)
                AFSEL15: u4,
            }),
            ///  port bit reset register
            BRR: mmio.Mmio(packed struct(u32) {
                ///  Port Reset bit
                BR0: u1,
                ///  Port Reset bit
                BR1: u1,
                ///  Port Reset bit
                BR2: u1,
                ///  Port Reset bit
                BR3: u1,
                ///  Port Reset bit
                BR4: u1,
                ///  Port Reset bit
                BR5: u1,
                ///  Port Reset bit
                BR6: u1,
                ///  Port Reset bit
                BR7: u1,
                ///  Port Reset bit
                BR8: u1,
                ///  Port Reset bit
                BR9: u1,
                ///  Port Reset bit
                BR10: u1,
                ///  Port Reset bit
                BR11: u1,
                ///  Port Reset bit
                BR12: u1,
                ///  Port Reset bit
                BR13: u1,
                ///  Port Reset bit
                BR14: u1,
                ///  Port Reset bit
                BR15: u1,
                padding: u16,
            }),
        };

        ///  System configuration controller
        pub const SYSCFG_ITLINE = extern struct {
            reserved128: [128]u8,
            ///  interrupt line 0 status register
            ITLINE0: mmio.Mmio(packed struct(u32) {
                ///  Window watchdog interrupt pending flag
                WWDG: u1,
                padding: u31,
            }),
            ///  interrupt line 1 status register
            ITLINE1: mmio.Mmio(packed struct(u32) {
                ///  PVD supply monitoring interrupt request pending (EXTI line 16).
                PVDOUT: u1,
                padding: u31,
            }),
            ///  interrupt line 2 status register
            ITLINE2: mmio.Mmio(packed struct(u32) {
                ///  TAMP
                TAMP: u1,
                ///  RTC
                RTC: u1,
                padding: u30,
            }),
            ///  interrupt line 3 status register
            ITLINE3: mmio.Mmio(packed struct(u32) {
                ///  FLASH_ITF
                FLASH_ITF: u1,
                ///  FLASH_ECC
                FLASH_ECC: u1,
                padding: u30,
            }),
            ///  interrupt line 4 status register
            ITLINE4: mmio.Mmio(packed struct(u32) {
                ///  RCC
                RCC: u1,
                padding: u31,
            }),
            ///  interrupt line 5 status register
            ITLINE5: mmio.Mmio(packed struct(u32) {
                ///  EXTI0
                EXTI0: u1,
                ///  EXTI1
                EXTI1: u1,
                padding: u30,
            }),
            ///  interrupt line 6 status register
            ITLINE6: mmio.Mmio(packed struct(u32) {
                ///  EXTI2
                EXTI2: u1,
                ///  EXTI3
                EXTI3: u1,
                padding: u30,
            }),
            ///  interrupt line 7 status register
            ITLINE7: mmio.Mmio(packed struct(u32) {
                ///  EXTI4
                EXTI4: u1,
                ///  EXTI5
                EXTI5: u1,
                ///  EXTI6
                EXTI6: u1,
                ///  EXTI7
                EXTI7: u1,
                ///  EXTI8
                EXTI8: u1,
                ///  EXTI9
                EXTI9: u1,
                ///  EXTI10
                EXTI10: u1,
                ///  EXTI11
                EXTI11: u1,
                ///  EXTI12
                EXTI12: u1,
                ///  EXTI13
                EXTI13: u1,
                ///  EXTI14
                EXTI14: u1,
                ///  EXTI15
                EXTI15: u1,
                padding: u20,
            }),
            reserved164: [4]u8,
            ///  interrupt line 9 status register
            ITLINE9: mmio.Mmio(packed struct(u32) {
                ///  DMA1_CH1
                DMA1_CH1: u1,
                padding: u31,
            }),
            ///  interrupt line 10 status register
            ITLINE10: mmio.Mmio(packed struct(u32) {
                ///  DMA1_CH1
                DMA1_CH2: u1,
                ///  DMA1_CH3
                DMA1_CH3: u1,
                padding: u30,
            }),
            ///  interrupt line 11 status register
            ITLINE11: mmio.Mmio(packed struct(u32) {
                ///  DMAMUX
                DMAMUX: u1,
                ///  DMA1_CH4
                DMA1_CH4: u1,
                ///  DMA1_CH5
                DMA1_CH5: u1,
                padding: u29,
            }),
            ///  interrupt line 12 status register
            ITLINE12: mmio.Mmio(packed struct(u32) {
                ///  ADC
                ADC: u1,
                padding: u31,
            }),
            ///  interrupt line 13 status register
            ITLINE13: mmio.Mmio(packed struct(u32) {
                ///  TIM1_CCU
                TIM1_CCU: u1,
                ///  TIM1_TRG
                TIM1_TRG: u1,
                ///  TIM1_UPD
                TIM1_UPD: u1,
                ///  TIM1_BRK
                TIM1_BRK: u1,
                padding: u28,
            }),
            ///  interrupt line 14 status register
            ITLINE14: mmio.Mmio(packed struct(u32) {
                ///  TIM1_CC
                TIM1_CC: u1,
                padding: u31,
            }),
            ///  interrupt line 15 status register
            ITLINE15: mmio.Mmio(packed struct(u32) {
                ///  TIM2
                TIM2: u1,
                padding: u31,
            }),
            ///  interrupt line 16 status register
            ITLINE16: mmio.Mmio(packed struct(u32) {
                ///  TIM3
                TIM3: u1,
                padding: u31,
            }),
            reserved204: [8]u8,
            ///  interrupt line 19 status register
            ITLINE19: mmio.Mmio(packed struct(u32) {
                ///  TIM14
                TIM14: u1,
                padding: u31,
            }),
            reserved212: [4]u8,
            ///  interrupt line 21 status register
            ITLINE21: mmio.Mmio(packed struct(u32) {
                ///  TIM16
                TIM16: u1,
                padding: u31,
            }),
            ///  interrupt line 22 status register
            ITLINE22: mmio.Mmio(packed struct(u32) {
                ///  TIM17
                TIM17: u1,
                padding: u31,
            }),
            ///  interrupt line 23 status register
            ITLINE23: mmio.Mmio(packed struct(u32) {
                ///  I2C1
                I2C1: u1,
                padding: u31,
            }),
            ///  interrupt line 24 status register
            ITLINE24: mmio.Mmio(packed struct(u32) {
                ///  I2C2
                I2C2: u1,
                padding: u31,
            }),
            ///  interrupt line 25 status register
            ITLINE25: mmio.Mmio(packed struct(u32) {
                ///  SPI1
                SPI1: u1,
                padding: u31,
            }),
            ///  interrupt line 26 status register
            ITLINE26: mmio.Mmio(packed struct(u32) {
                ///  SPI2
                SPI2: u1,
                padding: u31,
            }),
            ///  interrupt line 27 status register
            ITLINE27: mmio.Mmio(packed struct(u32) {
                ///  USART1
                USART1: u1,
                padding: u31,
            }),
            ///  interrupt line 28 status register
            ITLINE28: mmio.Mmio(packed struct(u32) {
                ///  USART2
                USART2: u1,
                padding: u31,
            }),
            ///  interrupt line 29 status register
            ITLINE29: mmio.Mmio(packed struct(u32) {
                reserved2: u2,
                ///  USART5
                USART5: u1,
                padding: u29,
            }),
        };

        ///  Floting point unit
        pub const FPU = extern struct {
            ///  Floating-point context control register
            FPCCR: mmio.Mmio(packed struct(u32) {
                ///  LSPACT
                LSPACT: u1,
                ///  USER
                USER: u1,
                reserved3: u1,
                ///  THREAD
                THREAD: u1,
                ///  HFRDY
                HFRDY: u1,
                ///  MMRDY
                MMRDY: u1,
                ///  BFRDY
                BFRDY: u1,
                reserved8: u1,
                ///  MONRDY
                MONRDY: u1,
                reserved30: u21,
                ///  LSPEN
                LSPEN: u1,
                ///  ASPEN
                ASPEN: u1,
            }),
            ///  Floating-point context address register
            FPCAR: mmio.Mmio(packed struct(u32) {
                reserved3: u3,
                ///  Location of unpopulated floating-point
                ADDRESS: u29,
            }),
            ///  Floating-point status control register
            FPSCR: mmio.Mmio(packed struct(u32) {
                ///  Invalid operation cumulative exception bit
                IOC: u1,
                ///  Division by zero cumulative exception bit.
                DZC: u1,
                ///  Overflow cumulative exception bit
                OFC: u1,
                ///  Underflow cumulative exception bit
                UFC: u1,
                ///  Inexact cumulative exception bit
                IXC: u1,
                reserved7: u2,
                ///  Input denormal cumulative exception bit.
                IDC: u1,
                reserved22: u14,
                ///  Rounding Mode control field
                RMode: u2,
                ///  Flush-to-zero mode control bit:
                FZ: u1,
                ///  Default NaN mode control bit
                DN: u1,
                ///  Alternative half-precision control bit
                AHP: u1,
                reserved28: u1,
                ///  Overflow condition code flag
                V: u1,
                ///  Carry condition code flag
                C: u1,
                ///  Zero condition code flag
                Z: u1,
                ///  Negative condition code flag
                N: u1,
            }),
        };

        ///  Floating point unit CPACR
        pub const FPU_CPACR = extern struct {
            ///  Coprocessor access control register
            CPACR: mmio.Mmio(packed struct(u32) {
                reserved20: u20,
                ///  CP
                CP: u4,
                padding: u8,
            }),
        };

        ///  Cyclic redundancy check calculation unit
        pub const CRC = extern struct {
            ///  Data register
            DR: mmio.Mmio(packed struct(u32) {
                ///  Data register bits
                DR: u32,
            }),
            ///  Independent data register
            IDR: mmio.Mmio(packed struct(u32) {
                ///  General-purpose 32-bit data register bits
                IDR: u32,
            }),
            ///  Control register
            CR: mmio.Mmio(packed struct(u32) {
                ///  RESET bit
                RESET: u1,
                reserved3: u2,
                ///  Polynomial size
                POLYSIZE: u2,
                ///  Reverse input data
                REV_IN: u2,
                ///  Reverse output data
                REV_OUT: u1,
                padding: u24,
            }),
            reserved16: [4]u8,
            ///  Initial CRC value
            INIT: mmio.Mmio(packed struct(u32) {
                ///  Programmable initial CRC value
                CRC_INIT: u32,
            }),
            ///  polynomial
            POL: mmio.Mmio(packed struct(u32) {
                ///  Programmable polynomial
                POL: u32,
            }),
        };

        ///  External interrupt/event controller
        pub const EXTI = extern struct {
            ///  EXTI rising trigger selection register
            RTSR1: mmio.Mmio(packed struct(u32) {
                ///  Rising trigger event configuration bit of Configurable Event input
                TR0: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR1: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR2: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR3: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR4: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR5: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR6: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR7: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR8: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR9: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR10: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR11: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR12: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR13: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR14: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR15: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR16: u1,
                padding: u15,
            }),
            ///  EXTI falling trigger selection register
            FTSR1: mmio.Mmio(packed struct(u32) {
                ///  Rising trigger event configuration bit of Configurable Event input
                TR0: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR1: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR2: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR3: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR4: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR5: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR6: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR7: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR8: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR9: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR10: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR11: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR12: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR13: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR14: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR15: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                TR16: u1,
                padding: u15,
            }),
            ///  EXTI software interrupt event register
            SWIER1: mmio.Mmio(packed struct(u32) {
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER0: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER1: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER2: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER3: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER4: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER5: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER6: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER7: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER8: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER9: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER10: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER11: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER12: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER13: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER14: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER15: u1,
                ///  Rising trigger event configuration bit of Configurable Event input
                SWIER16: u1,
                padding: u15,
            }),
            ///  EXTI rising edge pending register
            RPR1: mmio.Mmio(packed struct(u32) {
                ///  configurable event inputs x rising edge Pending bit.
                RPIF0: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF1: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF2: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF3: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF4: u1,
                ///  configurable event inputs x rising edge Pending bit
                RPIF5: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF6: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF7: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF8: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF9: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF10: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF11: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF12: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF13: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF14: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF15: u1,
                ///  configurable event inputs x rising edge Pending bit.
                RPIF16: u1,
                padding: u15,
            }),
            ///  EXTI falling edge pending register
            FPR1: mmio.Mmio(packed struct(u32) {
                ///  configurable event inputs x falling edge pending bit.
                FPIF0: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF1: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF2: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF3: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF4: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF5: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF6: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF7: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF8: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF9: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF10: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF11: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF12: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF13: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF14: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF15: u1,
                ///  configurable event inputs x falling edge pending bit.
                FPIF16: u1,
                padding: u15,
            }),
            reserved96: [76]u8,
            ///  EXTI external interrupt selection register
            EXTICR1: mmio.Mmio(packed struct(u32) {
                ///  GPIO port selection
                EXTI0_7: u8,
                ///  GPIO port selection
                EXTI8_15: u8,
                ///  GPIO port selection
                EXTI16_23: u8,
                ///  GPIO port selection
                EXTI24_31: u8,
            }),
            ///  EXTI external interrupt selection register
            EXTICR2: mmio.Mmio(packed struct(u32) {
                ///  GPIO port selection
                EXTI0_7: u8,
                ///  GPIO port selection
                EXTI8_15: u8,
                ///  GPIO port selection
                EXTI16_23: u8,
                ///  GPIO port selection
                EXTI24_31: u8,
            }),
            ///  EXTI external interrupt selection register
            EXTICR3: mmio.Mmio(packed struct(u32) {
                ///  GPIO port selection
                EXTI0_7: u8,
                ///  GPIO port selection
                EXTI8_15: u8,
                ///  GPIO port selection
                EXTI16_23: u8,
                ///  GPIO port selection
                EXTI24_31: u8,
            }),
            ///  EXTI external interrupt selection register
            EXTICR4: mmio.Mmio(packed struct(u32) {
                ///  GPIO port selection
                EXTI0_7: u8,
                ///  GPIO port selection
                EXTI8_15: u8,
                ///  GPIO port selection
                EXTI16_23: u8,
                ///  GPIO port selection
                EXTI24_31: u8,
            }),
            reserved128: [16]u8,
            ///  EXTI CPU wakeup with interrupt mask register
            IMR1: mmio.Mmio(packed struct(u32) {
                ///  CPU wakeup with interrupt mask on event input
                IM0: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM1: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM2: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM3: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM4: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM5: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM6: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM7: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM8: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM9: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM10: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM11: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM12: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM13: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM14: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM15: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM16: u1,
                reserved19: u2,
                ///  CPU wakeup with interrupt mask on event input
                IM19: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM20: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM21: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM22: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM23: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM24: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM25: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM26: u1,
                reserved28: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM28: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM29: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM30: u1,
                ///  CPU wakeup with interrupt mask on event input
                IM31: u1,
            }),
            ///  EXTI CPU wakeup with event mask register
            EMR1: mmio.Mmio(packed struct(u32) {
                ///  CPU wakeup with event mask on event input
                EM0: u1,
                ///  CPU wakeup with event mask on event input
                EM1: u1,
                ///  CPU wakeup with event mask on event input
                EM2: u1,
                ///  CPU wakeup with event mask on event input
                EM3: u1,
                ///  CPU wakeup with event mask on event input
                EM4: u1,
                ///  CPU wakeup with event mask on event input
                EM5: u1,
                ///  CPU wakeup with event mask on event input
                EM6: u1,
                ///  CPU wakeup with event mask on event input
                EM7: u1,
                ///  CPU wakeup with event mask on event input
                EM8: u1,
                ///  CPU wakeup with event mask on event input
                EM9: u1,
                ///  CPU wakeup with event mask on event input
                EM10: u1,
                ///  CPU wakeup with event mask on event input
                EM11: u1,
                ///  CPU wakeup with event mask on event input
                EM12: u1,
                ///  CPU wakeup with event mask on event input
                EM13: u1,
                ///  CPU wakeup with event mask on event input
                EM14: u1,
                ///  CPU wakeup with event mask on event input
                EM15: u1,
                ///  CPU wakeup with event mask on event input
                EM16: u1,
                reserved19: u2,
                ///  CPU wakeup with event mask on event input
                EM19: u1,
                reserved21: u1,
                ///  CPU wakeup with event mask on event input
                EM21: u1,
                reserved23: u1,
                ///  CPU wakeup with event mask on event input
                EM23: u1,
                reserved25: u1,
                ///  CPU wakeup with event mask on event input
                EM25: u1,
                ///  CPU wakeup with event mask on event input
                EM26: u1,
                reserved28: u1,
                ///  CPU wakeup with event mask on event input
                EM28: u1,
                ///  CPU wakeup with event mask on event input
                EM29: u1,
                ///  CPU wakeup with event mask on event input
                EM30: u1,
                ///  CPU wakeup with event mask on event input
                EM31: u1,
            }),
        };

        ///  General purpose timers
        pub const TIM16 = extern struct {
            ///  control register 1
            CR1: mmio.Mmio(packed struct(u32) {
                ///  Counter enable
                CEN: u1,
                ///  Update disable
                UDIS: u1,
                ///  Update request source
                URS: u1,
                ///  One-pulse mode
                OPM: u1,
                reserved7: u3,
                ///  Auto-reload preload enable
                ARPE: u1,
                ///  Clock division
                CKD: u2,
                reserved11: u1,
                ///  UIF status bit remapping
                UIFREMAP: u1,
                padding: u20,
            }),
            ///  control register 2
            CR2: mmio.Mmio(packed struct(u32) {
                ///  Capture/compare preloaded control
                CCPC: u1,
                reserved2: u1,
                ///  Capture/compare control update selection
                CCUS: u1,
                ///  Capture/compare DMA selection
                CCDS: u1,
                reserved8: u4,
                ///  Output Idle state 1
                OIS1: u1,
                ///  Output Idle state 1
                OIS1N: u1,
                padding: u22,
            }),
            reserved12: [4]u8,
            ///  DMA/Interrupt enable register
            DIER: mmio.Mmio(packed struct(u32) {
                ///  Update interrupt enable
                UIE: u1,
                ///  Capture/Compare 1 interrupt enable
                CC1IE: u1,
                reserved5: u3,
                ///  COM interrupt enable
                COMIE: u1,
                reserved7: u1,
                ///  Break interrupt enable
                BIE: u1,
                ///  Update DMA request enable
                UDE: u1,
                ///  Capture/Compare 1 DMA request enable
                CC1DE: u1,
                reserved13: u3,
                ///  COM DMA request enable
                COMDE: u1,
                padding: u18,
            }),
            ///  status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  Update interrupt flag
                UIF: u1,
                ///  Capture/compare 1 interrupt flag
                CC1IF: u1,
                reserved5: u3,
                ///  COM interrupt flag
                COMIF: u1,
                reserved7: u1,
                ///  Break interrupt flag
                BIF: u1,
                reserved9: u1,
                ///  Capture/Compare 1 overcapture flag
                CC1OF: u1,
                padding: u22,
            }),
            ///  event generation register
            EGR: mmio.Mmio(packed struct(u32) {
                ///  Update generation
                UG: u1,
                ///  Capture/compare 1 generation
                CC1G: u1,
                reserved5: u3,
                ///  Capture/Compare control update generation
                COMG: u1,
                reserved7: u1,
                ///  Break generation
                BG: u1,
                padding: u24,
            }),
            ///  capture/compare mode register (output mode)
            CCMR1_Output: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 1 selection
                CC1S: u2,
                ///  Output Compare 1 fast enable
                OC1FE: u1,
                ///  Output Compare 1 preload enable
                OC1PE: u1,
                /// Output Compare 1 mode
                /// Assumes OC1M_2 is set to 0
                OC1M: enum(u3) {
                    frozen = 0b000,
                    active = 0b001,
                    inactive = 0b010,
                    toggle = 0b011,
                    forced_inactive = 0b100,
                    forced_active = 0b101,
                    pwm_1 = 0b110,
                    pwm_2 = 0b111,
                },
                reserved16: u9,
                ///  Output Compare 1 mode
                OC1M_2: u1,
                padding: u15,
            }),
            reserved32: [4]u8,
            ///  capture/compare enable register
            CCER: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 1 output enable
                CC1E: u1,
                ///  Capture/Compare 1 output Polarity
                CC1P: u1,
                ///  Capture/Compare 1 complementary output enable
                CC1NE: u1,
                ///  Capture/Compare 1 output Polarity
                CC1NP: u1,
                padding: u28,
            }),
            ///  counter
            CNT: mmio.Mmio(packed struct(u32) {
                ///  counter value
                CNT: u16,
                reserved31: u15,
                ///  UIF Copy
                UIFCPY: u1,
            }),
            ///  prescaler
            PSC: mmio.Mmio(packed struct(u32) {
                ///  Prescaler value
                PSC: u16,
                padding: u16,
            }),
            ///  auto-reload register
            ARR: mmio.Mmio(packed struct(u32) {
                ///  Auto-reload value
                ARR: u16,
                padding: u16,
            }),
            ///  repetition counter register
            RCR: mmio.Mmio(packed struct(u32) {
                ///  Repetition counter value
                REP: u8,
                padding: u24,
            }),
            ///  capture/compare register 1
            CCR1: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 1 value
                CCR1: u16,
                padding: u16,
            }),
            reserved68: [12]u8,
            ///  break and dead-time register
            BDTR: mmio.Mmio(packed struct(u32) {
                ///  Dead-time generator setup
                DTG: u8,
                ///  Lock configuration
                LOCK: u2,
                ///  Off-state selection for Idle mode
                OSSI: u1,
                ///  Off-state selection for Run mode
                OSSR: u1,
                ///  Break enable
                BKE: u1,
                ///  Break polarity
                BKP: u1,
                ///  Automatic output enable
                AOE: u1,
                ///  Main output enable
                MOE: u1,
                ///  Break filter
                BKF: u4,
                reserved26: u6,
                ///  Break Disarm
                BKDSRM: u1,
                reserved28: u1,
                ///  Break Bidirectional
                BKBID: u1,
                padding: u3,
            }),
            ///  DMA control register
            DCR: mmio.Mmio(packed struct(u32) {
                ///  DMA base address
                DBA: u5,
                reserved8: u3,
                ///  DMA burst length
                DBL: u5,
                padding: u19,
            }),
            ///  DMA address for full transfer
            DMAR: mmio.Mmio(packed struct(u32) {
                ///  DMA register for burst accesses
                DMAB: u16,
                padding: u16,
            }),
            reserved96: [16]u8,
            ///  TIM17 option register 1
            AF1: mmio.Mmio(packed struct(u32) {
                ///  BRK BKIN input enable
                BKINE: u1,
                ///  BRK COMP1 enable
                BKCMP1E: u1,
                ///  BRK COMP2 enable
                BKCMP2E: u1,
                reserved8: u5,
                ///  BRK DFSDM_BREAK1 enable
                BKDFBK1E: u1,
                ///  BRK BKIN input polarity
                BKINP: u1,
                ///  BRK COMP1 input polarity
                BKCMP1P: u1,
                ///  BRK COMP2 input polarit
                BKCMP2P: u1,
                padding: u20,
            }),
            reserved104: [4]u8,
            ///  input selection register
            TISEL: mmio.Mmio(packed struct(u32) {
                ///  selects input
                TI1SEL: u4,
                padding: u28,
            }),
        };

        ///  System control block ACTLR
        pub const SCB_ACTRL = extern struct {
            ///  Auxiliary control register
            ACTRL: mmio.Mmio(packed struct(u32) {
                ///  DISMCYCINT
                DISMCYCINT: u1,
                ///  DISDEFWBUF
                DISDEFWBUF: u1,
                ///  DISFOLD
                DISFOLD: u1,
                reserved8: u5,
                ///  DISFPCA
                DISFPCA: u1,
                ///  DISOOFP
                DISOOFP: u1,
                padding: u22,
            }),
        };

        ///  Universal synchronous asynchronous receiver transmitter
        pub const USART1 = extern struct {
            ///  Control register 1
            CR1: mmio.Mmio(packed struct(u32) {
                ///  USART enable
                UE: u1,
                ///  USART enable in Stop mode
                UESM: u1,
                ///  Receiver enable
                RE: u1,
                ///  Transmitter enable
                TE: u1,
                ///  IDLE interrupt enable
                IDLEIE: u1,
                ///  RXNE interrupt enable
                RXNEIE: u1,
                ///  Transmission complete interrupt enable
                TCIE: u1,
                ///  interrupt enable
                TXEIE: u1,
                ///  PE interrupt enable
                PEIE: u1,
                ///  Parity selection
                PS: u1,
                ///  Parity control enable
                PCE: u1,
                ///  Receiver wakeup method
                WAKE: u1,
                ///  Word length
                M0: u1,
                ///  Mute mode enable
                MME: u1,
                ///  Character match interrupt enable
                CMIE: u1,
                ///  Oversampling mode
                OVER8: u1,
                ///  DEDT
                DEDT: u5,
                ///  DEAT
                DEAT: u5,
                ///  Receiver timeout interrupt enable
                RTOIE: u1,
                ///  End of Block interrupt enable
                EOBIE: u1,
                ///  Word length
                M1: u1,
                ///  FIFO mode enable
                FIFOEN: u1,
                ///  TXFIFO empty interrupt enable
                TXFEIE: u1,
                ///  RXFIFO Full interrupt enable
                RXFFIE: u1,
            }),
            ///  Control register 2
            CR2: mmio.Mmio(packed struct(u32) {
                ///  Synchronous Slave mode enable
                SLVEN: u1,
                reserved3: u2,
                ///  When the DSI_NSS bit is set, the NSS pin input will be ignored
                DIS_NSS: u1,
                ///  7-bit Address Detection/4-bit Address Detection
                ADDM7: u1,
                ///  LIN break detection length
                LBDL: u1,
                ///  LIN break detection interrupt enable
                LBDIE: u1,
                reserved8: u1,
                ///  Last bit clock pulse
                LBCL: u1,
                ///  Clock phase
                CPHA: u1,
                ///  Clock polarity
                CPOL: u1,
                ///  Clock enable
                CLKEN: u1,
                ///  STOP bits
                STOP: u2,
                ///  LIN mode enable
                LINEN: u1,
                ///  Swap TX/RX pins
                SWAP: u1,
                ///  RX pin active level inversion
                RXINV: u1,
                ///  TX pin active level inversion
                TXINV: u1,
                ///  Binary data inversion
                TAINV: u1,
                ///  Most significant bit first
                MSBFIRST: u1,
                ///  Auto baud rate enable
                ABREN: u1,
                ///  Auto baud rate mode
                ABRMOD: u2,
                ///  Receiver timeout enable
                RTOEN: u1,
                ///  Address of the USART node
                ADD0_3: u4,
                ///  Address of the USART node
                ADD4_7: u4,
            }),
            ///  Control register 3
            CR3: mmio.Mmio(packed struct(u32) {
                ///  Error interrupt enable
                EIE: u1,
                ///  Ir mode enable
                IREN: u1,
                ///  Ir low-power
                IRLP: u1,
                ///  Half-duplex selection
                HDSEL: u1,
                ///  Smartcard NACK enable
                NACK: u1,
                ///  Smartcard mode enable
                SCEN: u1,
                ///  DMA enable receiver
                DMAR: u1,
                ///  DMA enable transmitter
                DMAT: u1,
                ///  RTS enable
                RTSE: u1,
                ///  CTS enable
                CTSE: u1,
                ///  CTS interrupt enable
                CTSIE: u1,
                ///  One sample bit method enable
                ONEBIT: u1,
                ///  Overrun Disable
                OVRDIS: u1,
                ///  DMA Disable on Reception Error
                DDRE: u1,
                ///  Driver enable mode
                DEM: u1,
                ///  Driver enable polarity selection
                DEP: u1,
                reserved17: u1,
                ///  Smartcard auto-retry count
                SCARCNT: u3,
                ///  Wakeup from Stop mode interrupt flag selection
                WUS: u2,
                ///  Wakeup from Stop mode interrupt enable
                WUFIE: u1,
                ///  threshold interrupt enable
                TXFTIE: u1,
                ///  Tr Complete before guard time, interrupt enable
                TCBGTIE: u1,
                ///  Receive FIFO threshold configuration
                RXFTCFG: u3,
                ///  RXFIFO threshold interrupt enable
                RXFTIE: u1,
                ///  TXFIFO threshold configuration
                TXFTCFG: u3,
            }),
            ///  Baud rate register
            BRR: mmio.Mmio(packed struct(u32) {
                ///  BRR_0_3
                BRR_0_3: u4,
                ///  BRR_4_15
                BRR_4_15: u12,
                padding: u16,
            }),
            ///  Guard time and prescaler register
            GTPR: mmio.Mmio(packed struct(u32) {
                ///  Prescaler value
                PSC: u8,
                ///  Guard time value
                GT: u8,
                padding: u16,
            }),
            ///  Receiver timeout register
            RTOR: mmio.Mmio(packed struct(u32) {
                ///  Receiver timeout value
                RTO: u24,
                ///  Block Length
                BLEN: u8,
            }),
            ///  Request register
            RQR: mmio.Mmio(packed struct(u32) {
                ///  Auto baud rate request
                ABRRQ: u1,
                ///  Send break request
                SBKRQ: u1,
                ///  Mute mode request
                MMRQ: u1,
                ///  Receive data flush request
                RXFRQ: u1,
                ///  Transmit data flush request
                TXFRQ: u1,
                padding: u27,
            }),
            ///  Interrupt & status register
            ISR: mmio.Mmio(packed struct(u32) {
                ///  PE
                PE: u1,
                ///  FE
                FE: u1,
                ///  NF
                NF: u1,
                ///  ORE
                ORE: u1,
                ///  IDLE
                IDLE: u1,
                ///  RXNE
                RXNE: u1,
                ///  TC
                TC: u1,
                ///  TXE
                TXE: u1,
                ///  LBDF
                LBDF: u1,
                ///  CTSIF
                CTSIF: u1,
                ///  CTS
                CTS: u1,
                ///  RTOF
                RTOF: u1,
                ///  EOBF
                EOBF: u1,
                ///  SPI slave underrun error flag
                UDR: u1,
                ///  ABRE
                ABRE: u1,
                ///  ABRF
                ABRF: u1,
                ///  BUSY
                BUSY: u1,
                ///  CMF
                CMF: u1,
                ///  SBKF
                SBKF: u1,
                ///  RWU
                RWU: u1,
                ///  WUF
                WUF: u1,
                ///  TEACK
                TEACK: u1,
                ///  REACK
                REACK: u1,
                ///  TXFIFO Empty
                TXFE: u1,
                ///  RXFIFO Full
                RXFF: u1,
                ///  Transmission complete before guard time flag
                TCBGT: u1,
                ///  RXFIFO threshold flag
                RXFT: u1,
                ///  TXFIFO threshold flag
                TXFT: u1,
                padding: u4,
            }),
            ///  Interrupt flag clear register
            ICR: mmio.Mmio(packed struct(u32) {
                ///  Parity error clear flag
                PECF: u1,
                ///  Framing error clear flag
                FECF: u1,
                ///  Noise detected clear flag
                NCF: u1,
                ///  Overrun error clear flag
                ORECF: u1,
                ///  Idle line detected clear flag
                IDLECF: u1,
                ///  TXFIFO empty clear flag
                TXFECF: u1,
                ///  Transmission complete clear flag
                TCCF: u1,
                ///  Transmission complete before Guard time clear flag
                TCBGTCF: u1,
                ///  LIN break detection clear flag
                LBDCF: u1,
                ///  CTS clear flag
                CTSCF: u1,
                reserved11: u1,
                ///  Receiver timeout clear flag
                RTOCF: u1,
                ///  End of block clear flag
                EOBCF: u1,
                ///  SPI slave underrun clear flag
                UDRCF: u1,
                reserved17: u3,
                ///  Character match clear flag
                CMCF: u1,
                reserved20: u2,
                ///  Wakeup from Stop mode clear flag
                WUCF: u1,
                padding: u11,
            }),
            ///  Receive data register
            RDR: mmio.Mmio(packed struct(u32) {
                ///  Receive data value
                RDR: u9,
                padding: u23,
            }),
            ///  Transmit data register
            TDR: mmio.Mmio(packed struct(u32) {
                ///  Transmit data value
                TDR: u9,
                padding: u23,
            }),
            ///  Prescaler register
            PRESC: mmio.Mmio(packed struct(u32) {
                ///  Clock prescaler
                PRESCALER: u4,
                padding: u28,
            }),
        };

        ///  Nested vectored interrupt controller
        pub const NVIC_STIR = extern struct {
            ///  Software trigger interrupt register
            STIR: mmio.Mmio(packed struct(u32) {
                ///  Software generated interrupt ID
                INTID: u9,
                padding: u23,
            }),
        };

        ///  Serial peripheral interface/Inter-IC sound
        pub const SPI1 = extern struct {
            ///  control register 1
            CR1: mmio.Mmio(packed struct(u32) {
                ///  Clock phase
                CPHA: u1,
                ///  Clock polarity
                CPOL: u1,
                ///  Master selection
                MSTR: u1,
                ///  Baud rate control
                BR: u3,
                ///  SPI enable
                SPE: u1,
                ///  Frame format
                LSBFIRST: u1,
                ///  Internal slave select
                SSI: u1,
                ///  Software slave management
                SSM: u1,
                ///  Receive only
                RXONLY: u1,
                ///  Data frame format
                DFF: u1,
                ///  CRC transfer next
                CRCNEXT: u1,
                ///  Hardware CRC calculation enable
                CRCEN: u1,
                ///  Output enable in bidirectional mode
                BIDIOE: u1,
                ///  Bidirectional data mode enable
                BIDIMODE: u1,
                padding: u16,
            }),
            ///  control register 2
            CR2: mmio.Mmio(packed struct(u32) {
                ///  Rx buffer DMA enable
                RXDMAEN: u1,
                ///  Tx buffer DMA enable
                TXDMAEN: u1,
                ///  SS output enable
                SSOE: u1,
                ///  NSS pulse management
                NSSP: u1,
                ///  Frame format
                FRF: u1,
                ///  Error interrupt enable
                ERRIE: u1,
                ///  RX buffer not empty interrupt enable
                RXNEIE: u1,
                ///  Tx buffer empty interrupt enable
                TXEIE: u1,
                ///  Data size
                DS: u4,
                ///  FIFO reception threshold
                FRXTH: u1,
                ///  Last DMA transfer for reception
                LDMA_RX: u1,
                ///  Last DMA transfer for transmission
                LDMA_TX: u1,
                padding: u17,
            }),
            ///  status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  Receive buffer not empty
                RXNE: u1,
                ///  Transmit buffer empty
                TXE: u1,
                ///  Channel side
                CHSIDE: u1,
                ///  Underrun flag
                UDR: u1,
                ///  CRC error flag
                CRCERR: u1,
                ///  Mode fault
                MODF: u1,
                ///  Overrun flag
                OVR: u1,
                ///  Busy flag
                BSY: u1,
                ///  TI frame format error
                TIFRFE: u1,
                ///  FIFO reception level
                FRLVL: u2,
                ///  FIFO transmission level
                FTLVL: u2,
                padding: u19,
            }),
            ///  data register
            DR: mmio.Mmio(packed struct(u32) {
                ///  Data register
                DR: u16,
                padding: u16,
            }),
            ///  CRC polynomial register
            CRCPR: mmio.Mmio(packed struct(u32) {
                ///  CRC polynomial register
                CRCPOLY: u16,
                padding: u16,
            }),
            ///  RX CRC register
            RXCRCR: mmio.Mmio(packed struct(u32) {
                ///  Rx CRC register
                RxCRC: u16,
                padding: u16,
            }),
            ///  TX CRC register
            TXCRCR: mmio.Mmio(packed struct(u32) {
                ///  Tx CRC register
                TxCRC: u16,
                padding: u16,
            }),
            ///  configuration register
            I2SCFGR: mmio.Mmio(packed struct(u32) {
                ///  Channel length (number of bits per audio channel)
                CHLEN: u1,
                ///  Data length to be transferred
                DATLEN: u2,
                ///  Inactive state clock polarity
                CKPOL: u1,
                ///  standard selection
                I2SSTD: u2,
                reserved7: u1,
                ///  PCM frame synchronization
                PCMSYNC: u1,
                ///  I2S configuration mode
                I2SCFG: u2,
                ///  I2S enable
                SE2: u1,
                ///  I2S mode selection
                I2SMOD: u1,
                padding: u20,
            }),
            ///  prescaler register
            I2SPR: mmio.Mmio(packed struct(u32) {
                ///  linear prescaler
                I2SDIV: u8,
                ///  Odd factor for the prescaler
                ODD: u1,
                ///  Master clock output enable
                MCKOE: u1,
                padding: u22,
            }),
        };

        ///  MCU debug component
        pub const DBG = extern struct {
            ///  DBGMCU_IDCODE
            IDCODE: mmio.Mmio(packed struct(u32) {
                ///  Device identifier
                DEV_ID: u12,
                reserved16: u4,
                ///  Revision identifie
                REV_ID: u16,
            }),
            ///  Debug MCU configuration register
            CR: mmio.Mmio(packed struct(u32) {
                reserved1: u1,
                ///  Debug Stop mode
                DBG_STOP: u1,
                ///  Debug Standby mode
                DBG_STANDBY: u1,
                padding: u29,
            }),
            ///  Debug MCU APB1 freeze register1
            APB_FZ1: mmio.Mmio(packed struct(u32) {
                ///  TIM2 counter stopped when core is halted
                DBG_TIM2_STOP: u1,
                ///  TIM3 counter stopped when core is halted
                DBG_TIM3_STOP: u1,
                reserved10: u8,
                ///  RTC counter stopped when core is halted
                DBG_RTC_STOP: u1,
                ///  Window watchdog counter stopped when core is halted
                DBG_WWDG_STOP: u1,
                ///  Independent watchdog counter stopped when core is halted
                DBG_IWDG_STOP: u1,
                reserved21: u8,
                ///  I2C1 SMBUS timeout counter stopped when core is halted
                DBG_I2C1_STOP: u1,
                padding: u10,
            }),
            ///  Debug MCU APB1 freeze register 2
            APB_FZ2: mmio.Mmio(packed struct(u32) {
                reserved11: u11,
                ///  TIM1 counter stopped when core is halted
                DBG_TIM1_STOP: u1,
                reserved15: u3,
                ///  DBG_TIM14_STOP
                DBG_TIM14_STOP: u1,
                reserved17: u1,
                ///  DBG_TIM16_STOP
                DBG_TIM16_STOP: u1,
                ///  DBG_TIM17_STOP
                DBG_TIM17_STOP: u1,
                padding: u13,
            }),
        };

        ///  Advanced-timers
        pub const TIM1 = extern struct {
            ///  control register 1
            CR1: mmio.Mmio(packed struct(u32) {
                ///  Counter enable
                CEN: u1,
                ///  Update disable
                UDIS: u1,
                ///  Update request source
                URS: u1,
                ///  One-pulse mode
                OPM: u1,
                ///  Direction
                DIR: u1,
                ///  Center-aligned mode selection
                CMS: u2,
                ///  Auto-reload preload enable
                ARPE: u1,
                ///  Clock division
                CKD: u2,
                reserved11: u1,
                ///  UIF status bit remapping
                UIFREMAP: u1,
                padding: u20,
            }),
            ///  control register 2
            CR2: mmio.Mmio(packed struct(u32) {
                ///  Capture/compare preloaded control
                CCPC: u1,
                reserved2: u1,
                ///  Capture/compare control update selection
                CCUS: u1,
                ///  Capture/compare DMA selection
                CCDS: u1,
                ///  Master mode selection
                MMS: u3,
                ///  TI1 selection
                TI1S: u1,
                ///  Output Idle state 1
                OIS1: u1,
                ///  Output Idle state 1
                OIS1N: u1,
                ///  Output Idle state 2
                OIS2: u1,
                ///  Output Idle state 2
                OIS2N: u1,
                ///  Output Idle state 3
                OIS3: u1,
                ///  Output Idle state 3
                OIS3N: u1,
                ///  Output Idle state 4
                OIS4: u1,
                reserved16: u1,
                ///  Output Idle state 5 (OC5 output)
                OIS5: u1,
                reserved18: u1,
                ///  Output Idle state 6 (OC6 output)
                OIS6: u1,
                reserved20: u1,
                ///  Master mode selection 2
                MMS2: u4,
                padding: u8,
            }),
            ///  slave mode control register
            SMCR: mmio.Mmio(packed struct(u32) {
                ///  Slave mode selection
                SMS: u3,
                ///  OCREF clear selection
                OCCS: u1,
                ///  Trigger selection
                TS_4: u3,
                ///  Master/Slave mode
                MSM: u1,
                ///  External trigger filter
                ETF: u4,
                ///  External trigger prescaler
                ETPS: u2,
                ///  External clock enable
                ECE: u1,
                ///  External trigger polarity
                ETP: u1,
                ///  Slave mode selection - bit 3
                SMS_3: u1,
                reserved20: u3,
                ///  Trigger selection
                TS: u2,
                padding: u10,
            }),
            ///  DMA/Interrupt enable register
            DIER: mmio.Mmio(packed struct(u32) {
                ///  Update interrupt enable
                UIE: u1,
                ///  Capture/Compare 1 interrupt enable
                CC1IE: u1,
                ///  Capture/Compare 2 interrupt enable
                CC2IE: u1,
                ///  Capture/Compare 3 interrupt enable
                CC3IE: u1,
                ///  Capture/Compare 4 interrupt enable
                CC4IE: u1,
                ///  COM interrupt enable
                COMIE: u1,
                ///  Trigger interrupt enable
                TIE: u1,
                ///  Break interrupt enable
                BIE: u1,
                ///  Update DMA request enable
                UDE: u1,
                ///  Capture/Compare 1 DMA request enable
                CC1DE: u1,
                ///  Capture/Compare 2 DMA request enable
                CC2DE: u1,
                ///  Capture/Compare 3 DMA request enable
                CC3DE: u1,
                ///  Capture/Compare 4 DMA request enable
                CC4DE: u1,
                ///  COM DMA request enable
                COMDE: u1,
                ///  Trigger DMA request enable
                TDE: u1,
                padding: u17,
            }),
            ///  status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  Update interrupt flag
                UIF: u1,
                ///  Capture/compare 1 interrupt flag
                CC1IF: u1,
                ///  Capture/Compare 2 interrupt flag
                CC2IF: u1,
                ///  Capture/Compare 3 interrupt flag
                CC3IF: u1,
                ///  Capture/Compare 4 interrupt flag
                CC4IF: u1,
                ///  COM interrupt flag
                COMIF: u1,
                ///  Trigger interrupt flag
                TIF: u1,
                ///  Break interrupt flag
                BIF: u1,
                ///  Break 2 interrupt flag
                B2IF: u1,
                ///  Capture/Compare 1 overcapture flag
                CC1OF: u1,
                ///  Capture/compare 2 overcapture flag
                CC2OF: u1,
                ///  Capture/Compare 3 overcapture flag
                CC3OF: u1,
                ///  Capture/Compare 4 overcapture flag
                CC4OF: u1,
                ///  System Break interrupt flag
                SBIF: u1,
                reserved16: u2,
                ///  Compare 5 interrupt flag
                CC5IF: u1,
                ///  Compare 6 interrupt flag
                CC6IF: u1,
                padding: u14,
            }),
            ///  event generation register
            EGR: mmio.Mmio(packed struct(u32) {
                ///  Update generation
                UG: u1,
                ///  Capture/compare 1 generation
                CC1G: u1,
                ///  Capture/compare 2 generation
                CC2G: u1,
                ///  Capture/compare 3 generation
                CC3G: u1,
                ///  Capture/compare 4 generation
                CC4G: u1,
                ///  Capture/Compare control update generation
                COMG: u1,
                ///  Trigger generation
                TG: u1,
                ///  Break generation
                BG: u1,
                ///  Break 2 generation
                B2G: u1,
                padding: u23,
            }),
            ///  capture/compare mode register 1 (output mode)
            CCMR1_Output: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 1 selection
                CC1S: u2,
                ///  Output Compare 1 fast enable
                OC1FE: u1,
                ///  Output Compare 1 preload enable
                OC1PE: u1,
                ///  Output Compare 1 mode
                OC1M: u3,
                ///  Output Compare 1 clear enable
                OC1CE: u1,
                ///  Capture/Compare 2 selection
                CC2S: u2,
                ///  Output Compare 2 fast enable
                OC2FE: u1,
                ///  Output Compare 2 preload enable
                OC2PE: u1,
                ///  Output Compare 2 mode
                OC2M: u3,
                ///  Output Compare 2 clear enable
                OC2CE: u1,
                ///  Output Compare 1 mode - bit 3
                OC1M_3: u1,
                reserved24: u7,
                ///  Output Compare 2 mode - bit 3
                OC2M_3: u1,
                padding: u7,
            }),
            ///  capture/compare mode register 2 (output mode)
            CCMR2_Output: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 3 selection
                CC3S: u2,
                ///  Output compare 3 fast enable
                OC3FE: u1,
                ///  Output compare 3 preload enable
                OC3PE: u1,
                ///  Output compare 3 mode
                OC3M: u3,
                ///  Output compare 3 clear enable
                OC3CE: u1,
                ///  Capture/Compare 4 selection
                CC4S: u2,
                ///  Output compare 4 fast enable
                OC4FE: u1,
                ///  Output compare 4 preload enable
                OC4PE: u1,
                ///  Output compare 4 mode
                OC4M: u3,
                ///  Output compare 4 clear enable
                OC4CE: u1,
                ///  Output Compare 3 mode - bit 3
                OC3M_3: u1,
                reserved24: u7,
                ///  Output Compare 4 mode - bit 3
                OC4M_3: u1,
                padding: u7,
            }),
            ///  capture/compare enable register
            CCER: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 1 output enable
                CC1E: u1,
                ///  Capture/Compare 1 output Polarity
                CC1P: u1,
                ///  Capture/Compare 1 complementary output enable
                CC1NE: u1,
                ///  Capture/Compare 1 output Polarity
                CC1NP: u1,
                ///  Capture/Compare 2 output enable
                CC2E: u1,
                ///  Capture/Compare 2 output Polarity
                CC2P: u1,
                ///  Capture/Compare 2 complementary output enable
                CC2NE: u1,
                ///  Capture/Compare 2 output Polarity
                CC2NP: u1,
                ///  Capture/Compare 3 output enable
                CC3E: u1,
                ///  Capture/Compare 3 output Polarity
                CC3P: u1,
                ///  Capture/Compare 3 complementary output enable
                CC3NE: u1,
                ///  Capture/Compare 3 output Polarity
                CC3NP: u1,
                ///  Capture/Compare 4 output enable
                CC4E: u1,
                ///  Capture/Compare 3 output Polarity
                CC4P: u1,
                reserved15: u1,
                ///  Capture/Compare 4 complementary output polarity
                CC4NP: u1,
                ///  Capture/Compare 5 output enable
                CC5E: u1,
                ///  Capture/Compare 5 output polarity
                CC5P: u1,
                reserved20: u2,
                ///  Capture/Compare 6 output enable
                CC6E: u1,
                ///  Capture/Compare 6 output polarity
                CC6P: u1,
                padding: u10,
            }),
            ///  counter
            CNT: mmio.Mmio(packed struct(u32) {
                ///  counter value
                CNT: u16,
                reserved31: u15,
                ///  UIF copy
                UIFCPY: u1,
            }),
            ///  prescaler
            PSC: mmio.Mmio(packed struct(u32) {
                ///  Prescaler value
                PSC: u16,
                padding: u16,
            }),
            ///  auto-reload register
            ARR: mmio.Mmio(packed struct(u32) {
                ///  Auto-reload value
                ARR: u16,
                padding: u16,
            }),
            ///  repetition counter register
            RCR: mmio.Mmio(packed struct(u32) {
                ///  Repetition counter value
                REP: u16,
                padding: u16,
            }),
            ///  capture/compare register 1
            CCR1: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 1 value
                CCR1: u16,
                padding: u16,
            }),
            ///  capture/compare register 2
            CCR2: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 2 value
                CCR2: u16,
                padding: u16,
            }),
            ///  capture/compare register 3
            CCR3: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare value
                CCR3: u16,
                padding: u16,
            }),
            ///  capture/compare register 4
            CCR4: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare value
                CCR4: u16,
                padding: u16,
            }),
            ///  break and dead-time register
            BDTR: mmio.Mmio(packed struct(u32) {
                ///  Dead-time generator setup
                DTG: u8,
                ///  Lock configuration
                LOCK: u2,
                ///  Off-state selection for Idle mode
                OSSI: u1,
                ///  Off-state selection for Run mode
                OSSR: u1,
                ///  Break enable
                BKE: u1,
                ///  Break polarity
                BKP: u1,
                ///  Automatic output enable
                AOE: u1,
                ///  Main output enable
                MOE: u1,
                ///  Break filter
                BKF: u4,
                ///  Break 2 filter
                BK2F: u4,
                ///  Break 2 enable
                BK2E: u1,
                ///  Break 2 polarity
                BK2P: u1,
                ///  Break Disarm
                BKDSRM: u1,
                ///  Break2 Disarm
                BK2DSRM: u1,
                ///  Break Bidirectional
                BKBID: u1,
                ///  Break2 bidirectional
                BK2ID: u1,
                padding: u2,
            }),
            ///  DMA control register
            DCR: mmio.Mmio(packed struct(u32) {
                ///  DMA base address
                DBA: u5,
                reserved8: u3,
                ///  DMA burst length
                DBL: u5,
                padding: u19,
            }),
            ///  DMA address for full transfer
            DMAR: mmio.Mmio(packed struct(u32) {
                ///  DMA register for burst accesses
                DMAB: u16,
                padding: u16,
            }),
            ///  option register 1
            OR1: mmio.Mmio(packed struct(u32) {
                ///  Ocref_clr source selection
                OCREF_CLR: u1,
                padding: u31,
            }),
            ///  capture/compare mode register 2 (output mode)
            CCMR3_Output: mmio.Mmio(packed struct(u32) {
                reserved2: u2,
                ///  Output compare 5 fast enable
                OC5FE: u1,
                ///  Output compare 5 preload enable
                OC5PE: u1,
                ///  Output compare 5 mode
                OC5M: u3,
                ///  Output compare 5 clear enable
                OC5CE: u1,
                reserved10: u2,
                ///  Output compare 6 fast enable
                OC6FE: u1,
                ///  Output compare 6 preload enable
                OC6PE: u1,
                ///  Output compare 6 mode
                OC6M: u3,
                ///  Output compare 6 clear enable
                OC6CE: u1,
                ///  Output Compare 5 mode bit 3
                OC5M_bit3: u1,
                reserved24: u7,
                ///  Output Compare 6 mode bit 3
                OC6M_bit3: u1,
                padding: u7,
            }),
            ///  capture/compare register 4
            CCR5: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare value
                CCR5: u16,
                reserved29: u13,
                ///  Group Channel 5 and Channel 1
                GC5C1: u1,
                ///  Group Channel 5 and Channel 2
                GC5C2: u1,
                ///  Group Channel 5 and Channel 3
                GC5C3: u1,
            }),
            ///  capture/compare register 4
            CCR6: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare value
                CCR6: u16,
                padding: u16,
            }),
            ///  DMA address for full transfer
            AF1: mmio.Mmio(packed struct(u32) {
                ///  BRK BKIN input enable
                BKINE: u1,
                ///  BRK COMP1 enable
                BKCMP1E: u1,
                ///  BRK COMP2 enable
                BKCMP2E: u1,
                reserved9: u6,
                ///  BRK BKIN input polarity
                BKINP: u1,
                ///  BRK COMP1 input polarity
                BKCMP1P: u1,
                ///  BRK COMP2 input polarity
                BKCMP2P: u1,
                reserved14: u2,
                ///  ETR source selection
                ETRSEL: u3,
                padding: u15,
            }),
            ///  DMA address for full transfer
            AF2: mmio.Mmio(packed struct(u32) {
                ///  BRK2 BKIN input enable
                BK2INE: u1,
                ///  BRK2 COMP1 enable
                BK2CMP1E: u1,
                ///  BRK2 COMP2 enable
                BK2CMP2E: u1,
                reserved8: u5,
                ///  BRK2 DFSDM_BREAK0 enable
                BK2DFBK0E: u1,
                ///  BRK2 BKIN input polarity
                BK2INP: u1,
                ///  BRK2 COMP1 input polarity
                BK2CMP1P: u1,
                ///  BRK2 COMP2 input polarity
                BK2CMP2P: u1,
                padding: u20,
            }),
            ///  TIM1 timer input selection register
            TISEL: mmio.Mmio(packed struct(u32) {
                ///  selects TI1[0] to TI1[15] input
                TI1SEL3_0: u4,
                reserved8: u4,
                ///  selects TI2[0] to TI2[15] input
                TI2SEL3_0: u4,
                reserved16: u4,
                ///  selects TI3[0] to TI3[15] input
                TI3SEL3_0: u4,
                reserved24: u4,
                ///  selects TI4[0] to TI4[15] input
                TI4SEL3_0: u4,
                padding: u4,
            }),
        };

        ///  Analog to Digital Converter instance 1
        pub const ADC = extern struct {
            ///  ADC interrupt and status register
            ISR: mmio.Mmio(packed struct(u32) {
                ///  ADC ready flag
                ADRDY: u1,
                ///  ADC group regular end of sampling flag
                EOSMP: u1,
                ///  ADC group regular end of unitary conversion flag
                EOC: u1,
                ///  ADC group regular end of sequence conversions flag
                EOS: u1,
                ///  ADC group regular overrun flag
                OVR: u1,
                reserved7: u2,
                ///  ADC analog watchdog 1 flag
                AWD1: u1,
                ///  ADC analog watchdog 2 flag
                AWD2: u1,
                ///  ADC analog watchdog 3 flag
                AWD3: u1,
                reserved11: u1,
                ///  End Of Calibration flag
                EOCAL: u1,
                reserved13: u1,
                ///  Channel Configuration Ready flag
                CCRDY: u1,
                padding: u18,
            }),
            ///  ADC interrupt enable register
            IER: mmio.Mmio(packed struct(u32) {
                ///  ADC ready interrupt
                ADRDYIE: u1,
                ///  ADC group regular end of sampling interrupt
                EOSMPIE: u1,
                ///  ADC group regular end of unitary conversion interrupt
                EOCIE: u1,
                ///  ADC group regular end of sequence conversions interrupt
                EOSIE: u1,
                ///  ADC group regular overrun interrupt
                OVRIE: u1,
                reserved7: u2,
                ///  ADC analog watchdog 1 interrupt
                AWD1IE: u1,
                ///  ADC analog watchdog 2 interrupt
                AWD2IE: u1,
                ///  ADC analog watchdog 3 interrupt
                AWD3IE: u1,
                reserved11: u1,
                ///  End of calibration interrupt enable
                EOCALIE: u1,
                reserved13: u1,
                ///  Channel Configuration Ready Interrupt enable
                CCRDYIE: u1,
                padding: u18,
            }),
            ///  ADC control register
            CR: mmio.Mmio(packed struct(u32) {
                ///  ADC enable
                ADEN: u1,
                ///  ADC disable
                ADDIS: u1,
                ///  ADC group regular conversion start
                ADSTART: u1,
                reserved4: u1,
                ///  ADC group regular conversion stop
                ADSTP: u1,
                reserved28: u23,
                ///  ADC voltage regulator enable
                ADVREGEN: u1,
                reserved31: u2,
                ///  ADC calibration
                ADCAL: u1,
            }),
            ///  ADC configuration register 1
            CFGR1: mmio.Mmio(packed struct(u32) {
                ///  ADC DMA transfer enable
                DMAEN: u1,
                ///  ADC DMA transfer configuration
                DMACFG: u1,
                ///  Scan sequence direction
                SCANDIR: u1,
                ///  ADC data resolution
                RES: u2,
                ///  ADC data alignement
                ALIGN: u1,
                ///  ADC group regular external trigger source
                EXTSEL: u3,
                reserved10: u1,
                ///  ADC group regular external trigger polarity
                EXTEN: u2,
                ///  ADC group regular overrun configuration
                OVRMOD: u1,
                ///  ADC group regular continuous conversion mode
                CONT: u1,
                ///  Wait conversion mode
                WAIT: u1,
                ///  Auto-off mode
                AUTOFF: u1,
                ///  ADC group regular sequencer discontinuous mode
                DISCEN: u1,
                reserved21: u4,
                ///  Mode selection of the ADC_CHSELR register
                CHSELRMOD: u1,
                ///  ADC analog watchdog 1 monitoring a single channel or all channels
                AWD1SGL: u1,
                ///  ADC analog watchdog 1 enable on scope ADC group regular
                AWD1EN: u1,
                reserved26: u2,
                ///  ADC analog watchdog 1 monitored channel selection
                AWDCH1CH: u5,
                padding: u1,
            }),
            ///  ADC configuration register 2
            CFGR2: mmio.Mmio(packed struct(u32) {
                ///  ADC oversampler enable on scope ADC group regular
                OVSE: u1,
                reserved2: u1,
                ///  ADC oversampling ratio
                OVSR: u3,
                ///  ADC oversampling shift
                OVSS: u4,
                ///  ADC oversampling discontinuous mode (triggered mode) for ADC group regular
                TOVS: u1,
                reserved29: u19,
                ///  Low frequency trigger mode enable
                LFTRIG: u1,
                ///  ADC clock mode
                CKMODE: u2,
            }),
            ///  ADC sampling time register
            SMPR: mmio.Mmio(packed struct(u32) {
                ///  Sampling time selection
                SMP1: u3,
                reserved4: u1,
                ///  Sampling time selection
                SMP2: u3,
                reserved8: u1,
                ///  Channel sampling time selection
                SMPSEL: u19,
                padding: u5,
            }),
            reserved32: [8]u8,
            ///  watchdog threshold register
            AWD1TR: mmio.Mmio(packed struct(u32) {
                ///  ADC analog watchdog 1 threshold low
                LT1: u12,
                reserved16: u4,
                ///  ADC analog watchdog 1 threshold high
                HT1: u12,
                padding: u4,
            }),
            ///  watchdog threshold register
            AWD2TR: mmio.Mmio(packed struct(u32) {
                ///  ADC analog watchdog 2 threshold low
                LT2: u12,
                reserved16: u4,
                ///  ADC analog watchdog 2 threshold high
                HT2: u12,
                padding: u4,
            }),
            ///  channel selection register
            CHSELR: mmio.Mmio(packed struct(u32) {
                ///  Channel-x selection
                CHSEL: u19,
                padding: u13,
            }),
            ///  watchdog threshold register
            AWD3TR: mmio.Mmio(packed struct(u32) {
                ///  ADC analog watchdog 3 threshold high
                LT3: u12,
                reserved16: u4,
                ///  ADC analog watchdog 3 threshold high
                HT3: u12,
                padding: u4,
            }),
            reserved64: [16]u8,
            ///  ADC group regular conversion data register
            DR: mmio.Mmio(packed struct(u32) {
                ///  ADC group regular conversion data
                regularDATA: u16,
                padding: u16,
            }),
            reserved160: [92]u8,
            ///  ADC analog watchdog 2 configuration register
            AWD2CR: mmio.Mmio(packed struct(u32) {
                ///  ADC analog watchdog 2 monitored channel selection
                AWD2CH: u19,
                padding: u13,
            }),
            ///  ADC analog watchdog 3 configuration register
            AWD3CR: mmio.Mmio(packed struct(u32) {
                ///  ADC analog watchdog 3 monitored channel selection
                AWD3CH: u19,
                padding: u13,
            }),
            reserved180: [12]u8,
            ///  ADC calibration factors register
            CALFACT: mmio.Mmio(packed struct(u32) {
                ///  ADC calibration factor in single-ended mode
                CALFACT: u7,
                padding: u25,
            }),
            reserved776: [592]u8,
            ///  ADC common control register
            CCR: mmio.Mmio(packed struct(u32) {
                reserved18: u18,
                ///  ADC prescaler
                PRESC: u4,
                ///  VREFINT enable
                VREFEN: u1,
                ///  Temperature sensor enable
                TSEN: u1,
                ///  VBAT enable
                VBATEN: u1,
                padding: u7,
            }),
        };

        ///  System configuration controller
        pub const SYSCFG = extern struct {
            ///  SYSCFG configuration register 1
            CFGR1: mmio.Mmio(packed struct(u32) {
                ///  Memory mapping selection bits
                MEM_MODE: u2,
                reserved4: u2,
                ///  PA11 and PA12 remapping bit.
                PA11_PA12_RMP: u1,
                ///  IR output polarity selection
                IR_POL: u1,
                ///  IR Modulation Envelope signal selection.
                IR_MOD: u2,
                ///  I/O analog switch voltage booster enable
                BOOSTEN: u1,
                reserved16: u7,
                ///  Fast Mode Plus (FM+) driving capability activation bits
                I2C_PBx_FMP: u4,
                ///  FM+ driving capability activation for I2C1
                I2C1_FMP: u1,
                ///  FM+ driving capability activation for I2C2
                I2C2_FMP: u1,
                ///  Fast Mode Plus (FM+) driving capability activation bits
                I2C_PAx_FMP: u2,
                padding: u8,
            }),
            reserved24: [20]u8,
            ///  SYSCFG configuration register 1
            CFGR2: mmio.Mmio(packed struct(u32) {
                ///  Cortex-M0+ LOCKUP bit enable bit
                LOCKUP_LOCK: u1,
                ///  SRAM parity lock bit
                SRAM_PARITY_LOCK: u1,
                ///  PVD lock enable bit
                PVD_LOCK: u1,
                ///  ECC error lock bit
                ECC_LOCK: u1,
                reserved8: u4,
                ///  SRAM parity error flag
                SRAM_PEF: u1,
                reserved16: u7,
                ///  PA1_CDEN
                PA1_CDEN: u1,
                ///  PA3_CDEN
                PA3_CDEN: u1,
                ///  PA5_CDEN
                PA5_CDEN: u1,
                ///  PA6_CDEN
                PA6_CDEN: u1,
                ///  PA13_CDEN
                PA13_CDEN: u1,
                ///  PB0_CDEN
                PB0_CDEN: u1,
                ///  PB1_CDEN
                PB1_CDEN: u1,
                ///  PB2_CDEN
                PB2_CDEN: u1,
                padding: u8,
            }),
        };

        ///  Tamper and backup registers
        pub const TAMP = extern struct {
            ///  control register 1
            CR1: mmio.Mmio(packed struct(u32) {
                ///  TAMP1E
                TAMP1E: u1,
                ///  TAMP2E
                TAMP2E: u1,
                reserved16: u14,
                ///  ITAMP1E
                ITAMP1E: u1,
                reserved18: u1,
                ///  ITAMP3E
                ITAMP3E: u1,
                ///  ITAMP4E
                ITAMP4E: u1,
                ///  ITAMP5E
                ITAMP5E: u1,
                ///  ITAMP6E
                ITAMP6E: u1,
                padding: u10,
            }),
            ///  control register 2
            CR2: mmio.Mmio(packed struct(u32) {
                ///  TAMP1NOER
                TAMP1NOER: u1,
                ///  TAMP2NOER
                TAMP2NOER: u1,
                reserved16: u14,
                ///  TAMP1MSK
                TAMP1MSK: u1,
                ///  TAMP2MSK
                TAMP2MSK: u1,
                reserved24: u6,
                ///  TAMP1TRG
                TAMP1TRG: u1,
                ///  TAMP2TRG
                TAMP2TRG: u1,
                padding: u6,
            }),
            reserved12: [4]u8,
            ///  TAMP filter control register
            FLTCR: mmio.Mmio(packed struct(u32) {
                ///  TAMPFREQ
                TAMPFREQ: u3,
                ///  TAMPFLT
                TAMPFLT: u2,
                ///  TAMPPRCH
                TAMPPRCH: u2,
                ///  TAMPPUDIS
                TAMPPUDIS: u1,
                padding: u24,
            }),
            reserved44: [28]u8,
            ///  TAMP interrupt enable register
            IER: mmio.Mmio(packed struct(u32) {
                ///  TAMP1IE
                TAMP1IE: u1,
                ///  TAMP2IE
                TAMP2IE: u1,
                reserved16: u14,
                ///  ITAMP1IE
                ITAMP1IE: u1,
                reserved18: u1,
                ///  ITAMP3IE
                ITAMP3IE: u1,
                ///  ITAMP4IE
                ITAMP4IE: u1,
                ///  ITAMP5IE
                ITAMP5IE: u1,
                ///  ITAMP6IE
                ITAMP6IE: u1,
                padding: u10,
            }),
            ///  TAMP status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  TAMP1F
                TAMP1F: u1,
                ///  TAMP2F
                TAMP2F: u1,
                reserved16: u14,
                ///  ITAMP1F
                ITAMP1F: u1,
                reserved18: u1,
                ///  ITAMP3F
                ITAMP3F: u1,
                ///  ITAMP4F
                ITAMP4F: u1,
                ///  ITAMP5F
                ITAMP5F: u1,
                ///  ITAMP6F
                ITAMP6F: u1,
                ///  ITAMP7F
                ITAMP7F: u1,
                padding: u9,
            }),
            ///  TAMP masked interrupt status register
            MISR: mmio.Mmio(packed struct(u32) {
                ///  TAMP1MF:
                TAMP1MF: u1,
                ///  TAMP2MF
                TAMP2MF: u1,
                reserved16: u14,
                ///  ITAMP1MF
                ITAMP1MF: u1,
                reserved18: u1,
                ///  ITAMP3MF
                ITAMP3MF: u1,
                ///  ITAMP4MF
                ITAMP4MF: u1,
                ///  ITAMP5MF
                ITAMP5MF: u1,
                ///  ITAMP6MF
                ITAMP6MF: u1,
                padding: u10,
            }),
            reserved60: [4]u8,
            ///  TAMP status clear register
            SCR: mmio.Mmio(packed struct(u32) {
                ///  CTAMP1F
                CTAMP1F: u1,
                ///  CTAMP2F
                CTAMP2F: u1,
                reserved16: u14,
                ///  CITAMP1F
                CITAMP1F: u1,
                reserved18: u1,
                ///  CITAMP3F
                CITAMP3F: u1,
                ///  CITAMP4F
                CITAMP4F: u1,
                ///  CITAMP5F
                CITAMP5F: u1,
                ///  CITAMP6F
                CITAMP6F: u1,
                ///  CITAMP7F
                CITAMP7F: u1,
                padding: u9,
            }),
            reserved256: [192]u8,
            ///  TAMP backup register
            BKP0R: mmio.Mmio(packed struct(u32) {
                ///  BKP
                BKP: u32,
            }),
            ///  TAMP backup register
            BKP1R: mmio.Mmio(packed struct(u32) {
                ///  BKP
                BKP: u32,
            }),
            ///  TAMP backup register
            BKP2R: mmio.Mmio(packed struct(u32) {
                ///  BKP
                BKP: u32,
            }),
            ///  TAMP backup register
            BKP3R: mmio.Mmio(packed struct(u32) {
                ///  BKP
                BKP: u32,
            }),
            ///  TAMP backup register
            BKP4R: mmio.Mmio(packed struct(u32) {
                ///  BKP
                BKP: u32,
            }),
        };

        ///  Inter-integrated circuit
        pub const I2C1 = extern struct {
            ///  Control register 1
            CR1: mmio.Mmio(packed struct(u32) {
                ///  Peripheral enable
                PE: u1,
                ///  TX Interrupt enable
                TXIE: u1,
                ///  RX Interrupt enable
                RXIE: u1,
                ///  Address match interrupt enable (slave only)
                ADDRIE: u1,
                ///  Not acknowledge received interrupt enable
                NACKIE: u1,
                ///  STOP detection Interrupt enable
                STOPIE: u1,
                ///  Transfer Complete interrupt enable
                TCIE: u1,
                ///  Error interrupts enable
                ERRIE: u1,
                ///  Digital noise filter
                DNF: u4,
                ///  Analog noise filter OFF
                ANFOFF: u1,
                reserved14: u1,
                ///  DMA transmission requests enable
                TXDMAEN: u1,
                ///  DMA reception requests enable
                RXDMAEN: u1,
                ///  Slave byte control
                SBC: u1,
                ///  Clock stretching disable
                NOSTRETCH: u1,
                ///  Wakeup from STOP enable
                WUPEN: u1,
                ///  General call enable
                GCEN: u1,
                ///  SMBus Host address enable
                SMBHEN: u1,
                ///  SMBus Device Default address enable
                SMBDEN: u1,
                ///  SMBUS alert enable
                ALERTEN: u1,
                ///  PEC enable
                PECEN: u1,
                padding: u8,
            }),
            ///  Control register 2
            CR2: mmio.Mmio(packed struct(u32) {
                ///  Slave address bit (master mode)
                SADD: u10,
                ///  Transfer direction (master mode)
                RD_WRN: u1,
                ///  10-bit addressing mode (master mode)
                ADD10: u1,
                ///  10-bit address header only read direction (master receiver mode)
                HEAD10R: u1,
                ///  Start generation
                START: u1,
                ///  Stop generation (master mode)
                STOP: u1,
                ///  NACK generation (slave mode)
                NACK: u1,
                ///  Number of bytes
                NBYTES: u8,
                ///  NBYTES reload mode
                RELOAD: u1,
                ///  Automatic end mode (master mode)
                AUTOEND: u1,
                ///  Packet error checking byte
                PECBYTE: u1,
                padding: u5,
            }),
            ///  Own address register 1
            OAR1: mmio.Mmio(packed struct(u32) {
                ///  Interface address
                OA1_0: u1,
                ///  Interface address
                OA1_7_1: u7,
                ///  Interface address
                OA1_8_9: u2,
                ///  Own Address 1 10-bit mode
                OA1MODE: u1,
                reserved15: u4,
                ///  Own Address 1 enable
                OA1EN: u1,
                padding: u16,
            }),
            ///  Own address register 2
            OAR2: mmio.Mmio(packed struct(u32) {
                reserved1: u1,
                ///  Interface address
                OA2: u7,
                ///  Own Address 2 masks
                OA2MSK: u3,
                reserved15: u4,
                ///  Own Address 2 enable
                OA2EN: u1,
                padding: u16,
            }),
            ///  Timing register
            TIMINGR: mmio.Mmio(packed struct(u32) {
                ///  SCL low period (master mode)
                SCLL: u8,
                ///  SCL high period (master mode)
                SCLH: u8,
                ///  Data hold time
                SDADEL: u4,
                ///  Data setup time
                SCLDEL: u4,
                reserved28: u4,
                ///  Timing prescaler
                PRESC: u4,
            }),
            ///  Status register 1
            TIMEOUTR: mmio.Mmio(packed struct(u32) {
                ///  Bus timeout A
                TIMEOUTA: u12,
                ///  Idle clock timeout detection
                TIDLE: u1,
                reserved15: u2,
                ///  Clock timeout enable
                TIMOUTEN: u1,
                ///  Bus timeout B
                TIMEOUTB: u12,
                reserved31: u3,
                ///  Extended clock timeout enable
                TEXTEN: u1,
            }),
            ///  Interrupt and Status register
            ISR: mmio.Mmio(packed struct(u32) {
                ///  Transmit data register empty (transmitters)
                TXE: u1,
                ///  Transmit interrupt status (transmitters)
                TXIS: u1,
                ///  Receive data register not empty (receivers)
                RXNE: u1,
                ///  Address matched (slave mode)
                ADDR: u1,
                ///  Not acknowledge received flag
                NACKF: u1,
                ///  Stop detection flag
                STOPF: u1,
                ///  Transfer Complete (master mode)
                TC: u1,
                ///  Transfer Complete Reload
                TCR: u1,
                ///  Bus error
                BERR: u1,
                ///  Arbitration lost
                ARLO: u1,
                ///  Overrun/Underrun (slave mode)
                OVR: u1,
                ///  PEC Error in reception
                PECERR: u1,
                ///  Timeout or t_low detection flag
                TIMEOUT: u1,
                ///  SMBus alert
                ALERT: u1,
                reserved15: u1,
                ///  Bus busy
                BUSY: u1,
                ///  Transfer direction (Slave mode)
                DIR: u1,
                ///  Address match code (Slave mode)
                ADDCODE: u7,
                padding: u8,
            }),
            ///  Interrupt clear register
            ICR: mmio.Mmio(packed struct(u32) {
                reserved3: u3,
                ///  Address Matched flag clear
                ADDRCF: u1,
                ///  Not Acknowledge flag clear
                NACKCF: u1,
                ///  Stop detection flag clear
                STOPCF: u1,
                reserved8: u2,
                ///  Bus error flag clear
                BERRCF: u1,
                ///  Arbitration lost flag clear
                ARLOCF: u1,
                ///  Overrun/Underrun flag clear
                OVRCF: u1,
                ///  PEC Error flag clear
                PECCF: u1,
                ///  Timeout detection flag clear
                TIMOUTCF: u1,
                ///  Alert flag clear
                ALERTCF: u1,
                padding: u18,
            }),
            ///  PEC register
            PECR: mmio.Mmio(packed struct(u32) {
                ///  Packet error checking register
                PEC: u8,
                padding: u24,
            }),
            ///  Receive data register
            RXDR: mmio.Mmio(packed struct(u32) {
                ///  8-bit receive data
                RXDATA: u8,
                padding: u24,
            }),
            ///  Transmit data register
            TXDR: mmio.Mmio(packed struct(u32) {
                ///  8-bit transmit data
                TXDATA: u8,
                padding: u24,
            }),
        };

        ///  System configuration controller
        pub const VREFBUF = extern struct {
            ///  VREFBUF control and status register
            CSR: mmio.Mmio(packed struct(u32) {
                ///  Voltage reference buffer mode enable This bit is used to enable the voltage reference buffer mode.
                ENVR: u1,
                ///  High impedance mode This bit controls the analog switch to connect or not the VREF+ pin. Refer to Table196: VREF buffer modes for the mode descriptions depending on ENVR bit configuration.
                HIZ: u1,
                reserved3: u1,
                ///  Voltage reference buffer ready
                VRR: u1,
                ///  Voltage reference scale These bits select the value generated by the voltage reference buffer. Other: Reserved
                VRS: u3,
                padding: u25,
            }),
            ///  VREFBUF calibration control register
            CCR: mmio.Mmio(packed struct(u32) {
                ///  Trimming code These bits are automatically initialized after reset with the trimming value stored in the Flash memory during the production test. Writing into these bits allows to tune the internal reference buffer voltage.
                TRIM: u6,
                padding: u26,
            }),
        };

        ///  Real-time clock
        pub const RTC = extern struct {
            ///  time register
            TR: mmio.Mmio(packed struct(u32) {
                ///  Second units in BCD format
                SU: u4,
                ///  Second tens in BCD format
                ST: u3,
                reserved8: u1,
                ///  Minute units in BCD format
                MNU: u4,
                ///  Minute tens in BCD format
                MNT: u3,
                reserved16: u1,
                ///  Hour units in BCD format
                HU: u4,
                ///  Hour tens in BCD format
                HT: u2,
                ///  AM/PM notation
                PM: u1,
                padding: u9,
            }),
            ///  date register
            DR: mmio.Mmio(packed struct(u32) {
                ///  Date units in BCD format
                DU: u4,
                ///  Date tens in BCD format
                DT: u2,
                reserved8: u2,
                ///  Month units in BCD format
                MU: u4,
                ///  Month tens in BCD format
                MT: u1,
                ///  Week day units
                WDU: u3,
                ///  Year units in BCD format
                YU: u4,
                ///  Year tens in BCD format
                YT: u4,
                padding: u8,
            }),
            ///  sub second register
            SSR: mmio.Mmio(packed struct(u32) {
                ///  Sub second value
                SS: u16,
                padding: u16,
            }),
            ///  initialization and status register
            ICSR: mmio.Mmio(packed struct(u32) {
                ///  Alarm A write flag
                ALRAWF: u1,
                ///  Alarm B write flag
                ALRBWF: u1,
                ///  Wakeup timer write flag
                WUTWF: u1,
                ///  Shift operation pending
                SHPF: u1,
                ///  Initialization status flag
                INITS: u1,
                ///  Registers synchronization flag
                RSF: u1,
                ///  Initialization flag
                INITF: u1,
                ///  Initialization mode
                INIT: u1,
                reserved16: u8,
                ///  Recalibration pending Flag
                RECALPF: u1,
                padding: u15,
            }),
            ///  prescaler register
            PRER: mmio.Mmio(packed struct(u32) {
                ///  Synchronous prescaler factor
                PREDIV_S: u15,
                reserved16: u1,
                ///  Asynchronous prescaler factor
                PREDIV_A: u7,
                padding: u9,
            }),
            ///  wakeup timer register
            WUTR: mmio.Mmio(packed struct(u32) {
                ///  Wakeup auto-reload value bits
                WUT: u16,
                padding: u16,
            }),
            ///  control register
            CR: mmio.Mmio(packed struct(u32) {
                ///  WUCKSEL
                WUCKSEL: u3,
                ///  TSEDGE
                TSEDGE: u1,
                ///  REFCKON
                REFCKON: u1,
                ///  BYPSHAD
                BYPSHAD: u1,
                ///  FMT
                FMT: u1,
                reserved8: u1,
                ///  ALRAE
                ALRAE: u1,
                ///  ALRBE
                ALRBE: u1,
                ///  WUTE
                WUTE: u1,
                ///  TSE
                TSE: u1,
                ///  ALRAIE
                ALRAIE: u1,
                ///  ALRBIE
                ALRBIE: u1,
                ///  WUTIE
                WUTIE: u1,
                ///  TSIE
                TSIE: u1,
                ///  ADD1H
                ADD1H: u1,
                ///  SUB1H
                SUB1H: u1,
                ///  BKP
                BKP: u1,
                ///  COSEL
                COSEL: u1,
                ///  POL
                POL: u1,
                ///  OSEL
                OSEL: u2,
                ///  COE
                COE: u1,
                ///  ITSE
                ITSE: u1,
                ///  TAMPTS
                TAMPTS: u1,
                ///  TAMPOE
                TAMPOE: u1,
                reserved29: u2,
                ///  TAMPALRM_PU
                TAMPALRM_PU: u1,
                ///  TAMPALRM_TYPE
                TAMPALRM_TYPE: u1,
                ///  OUT2EN
                OUT2EN: u1,
            }),
            reserved36: [8]u8,
            ///  write protection register
            WPR: mmio.Mmio(packed struct(u32) {
                ///  Write protection key
                KEY: u8,
                padding: u24,
            }),
            ///  calibration register
            CALR: mmio.Mmio(packed struct(u32) {
                ///  Calibration minus
                CALM: u9,
                reserved13: u4,
                ///  Use a 16-second calibration cycle period
                CALW16: u1,
                ///  Use an 8-second calibration cycle period
                CALW8: u1,
                ///  Increase frequency of RTC by 488.5 ppm
                CALP: u1,
                padding: u16,
            }),
            ///  shift control register
            SHIFTR: mmio.Mmio(packed struct(u32) {
                ///  Subtract a fraction of a second
                SUBFS: u15,
                reserved31: u16,
                ///  Add one second
                ADD1S: u1,
            }),
            ///  time stamp time register
            TSTR: mmio.Mmio(packed struct(u32) {
                ///  Second units in BCD format
                SU: u4,
                ///  Second tens in BCD format
                ST: u3,
                reserved8: u1,
                ///  Minute units in BCD format
                MNU: u4,
                ///  Minute tens in BCD format
                MNT: u3,
                reserved16: u1,
                ///  Hour units in BCD format
                HU: u4,
                ///  Hour tens in BCD format
                HT: u2,
                ///  AM/PM notation
                PM: u1,
                padding: u9,
            }),
            ///  time stamp date register
            TSDR: mmio.Mmio(packed struct(u32) {
                ///  Date units in BCD format
                DU: u4,
                ///  Date tens in BCD format
                DT: u2,
                reserved8: u2,
                ///  Month units in BCD format
                MU: u4,
                ///  Month tens in BCD format
                MT: u1,
                ///  Week day units
                WDU: u3,
                padding: u16,
            }),
            ///  timestamp sub second register
            TSSSR: mmio.Mmio(packed struct(u32) {
                ///  Sub second value
                SS: u16,
                padding: u16,
            }),
            reserved64: [4]u8,
            ///  alarm A register
            ALRMAR: mmio.Mmio(packed struct(u32) {
                ///  Second units in BCD format
                SU: u4,
                ///  Second tens in BCD format
                ST: u3,
                ///  Alarm A seconds mask
                MSK1: u1,
                ///  Minute units in BCD format
                MNU: u4,
                ///  Minute tens in BCD format
                MNT: u3,
                ///  Alarm A minutes mask
                MSK2: u1,
                ///  Hour units in BCD format
                HU: u4,
                ///  Hour tens in BCD format
                HT: u2,
                ///  AM/PM notation
                PM: u1,
                ///  Alarm A hours mask
                MSK3: u1,
                ///  Date units or day in BCD format
                DU: u4,
                ///  Date tens in BCD format
                DT: u2,
                ///  Week day selection
                WDSEL: u1,
                ///  Alarm A date mask
                MSK4: u1,
            }),
            ///  alarm A sub second register
            ALRMASSR: mmio.Mmio(packed struct(u32) {
                ///  Sub seconds value
                SS: u15,
                reserved24: u9,
                ///  Mask the most-significant bits starting at this bit
                MASKSS: u4,
                padding: u4,
            }),
            ///  alarm B register
            ALRMBR: mmio.Mmio(packed struct(u32) {
                ///  Second units in BCD format
                SU: u4,
                ///  Second tens in BCD format
                ST: u3,
                ///  Alarm B seconds mask
                MSK1: u1,
                ///  Minute units in BCD format
                MNU: u4,
                ///  Minute tens in BCD format
                MNT: u3,
                ///  Alarm B minutes mask
                MSK2: u1,
                ///  Hour units in BCD format
                HU: u4,
                ///  Hour tens in BCD format
                HT: u2,
                ///  AM/PM notation
                PM: u1,
                ///  Alarm B hours mask
                MSK3: u1,
                ///  Date units or day in BCD format
                DU: u4,
                ///  Date tens in BCD format
                DT: u2,
                ///  Week day selection
                WDSEL: u1,
                ///  Alarm B date mask
                MSK4: u1,
            }),
            ///  alarm B sub second register
            ALRMBSSR: mmio.Mmio(packed struct(u32) {
                ///  Sub seconds value
                SS: u15,
                reserved24: u9,
                ///  Mask the most-significant bits starting at this bit
                MASKSS: u4,
                padding: u4,
            }),
            ///  status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  ALRAF
                ALRAF: u1,
                ///  ALRBF
                ALRBF: u1,
                ///  WUTF
                WUTF: u1,
                ///  TSF
                TSF: u1,
                ///  TSOVF
                TSOVF: u1,
                ///  ITSF
                ITSF: u1,
                padding: u26,
            }),
            ///  masked interrupt status register
            MISR: mmio.Mmio(packed struct(u32) {
                ///  ALRAMF
                ALRAMF: u1,
                ///  ALRBMF
                ALRBMF: u1,
                ///  WUTMF
                WUTMF: u1,
                ///  TSMF
                TSMF: u1,
                ///  TSOVMF
                TSOVMF: u1,
                ///  ITSMF
                ITSMF: u1,
                padding: u26,
            }),
            reserved92: [4]u8,
            ///  status clear register
            SCR: mmio.Mmio(packed struct(u32) {
                ///  CALRAF
                CALRAF: u1,
                ///  CALRBF
                CALRBF: u1,
                ///  CWUTF
                CWUTF: u1,
                ///  CTSF
                CTSF: u1,
                ///  CTSOVF
                CTSOVF: u1,
                ///  CITSF
                CITSF: u1,
                padding: u26,
            }),
        };

        ///  General purpose timers
        pub const TIM14 = extern struct {
            ///  control register 1
            CR1: mmio.Mmio(packed struct(u32) {
                ///  Counter enable
                CEN: u1,
                ///  Update disable
                UDIS: u1,
                ///  Update request source
                URS: u1,
                ///  One-pulse mode
                OPM: u1,
                reserved7: u3,
                ///  Auto-reload preload enable
                ARPE: u1,
                ///  Clock division
                CKD: u2,
                reserved11: u1,
                ///  UIF status bit remapping
                UIFREMAP: u1,
                padding: u20,
            }),
            reserved12: [8]u8,
            ///  DMA/Interrupt enable register
            DIER: mmio.Mmio(packed struct(u32) {
                ///  Update interrupt enable
                UIE: u1,
                ///  Capture/Compare 1 interrupt enable
                CC1IE: u1,
                padding: u30,
            }),
            ///  status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  Update interrupt flag
                UIF: u1,
                ///  Capture/compare 1 interrupt flag
                CC1IF: u1,
                reserved9: u7,
                ///  Capture/Compare 1 overcapture flag
                CC1OF: u1,
                padding: u22,
            }),
            ///  event generation register
            EGR: mmio.Mmio(packed struct(u32) {
                ///  Update generation
                UG: u1,
                ///  Capture/compare 1 generation
                CC1G: u1,
                padding: u30,
            }),
            ///  capture/compare mode register 1 (output mode)
            CCMR1_Output: mmio.Mmio(packed struct(u32) {
                ///  CC1S
                CC1S: u2,
                ///  OC1FE
                OC1FE: u1,
                ///  OC1PE
                OC1PE: u1,
                ///  OC1M
                OC1M: u3,
                ///  OC1CE
                OC1CE: u1,
                reserved16: u8,
                ///  Output Compare 1 mode - bit 3
                OC1M_3: u1,
                padding: u15,
            }),
            reserved32: [4]u8,
            ///  capture/compare enable register
            CCER: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 1 output enable
                CC1E: u1,
                ///  Capture/Compare 1 output Polarity
                CC1P: u1,
                reserved3: u1,
                ///  Capture/Compare 1 output Polarity
                CC1NP: u1,
                padding: u28,
            }),
            ///  counter
            CNT: mmio.Mmio(packed struct(u32) {
                ///  low counter value
                CNT: u16,
                reserved31: u15,
                ///  UIF Copy
                UIFCPY: u1,
            }),
            ///  prescaler
            PSC: mmio.Mmio(packed struct(u32) {
                ///  Prescaler value
                PSC: u16,
                padding: u16,
            }),
            ///  auto-reload register
            ARR: mmio.Mmio(packed struct(u32) {
                ///  Low Auto-reload value
                ARR: u16,
                padding: u16,
            }),
            reserved52: [4]u8,
            ///  capture/compare register 1
            CCR1: mmio.Mmio(packed struct(u32) {
                ///  Low Capture/Compare 1 value
                CCR1: u16,
                padding: u16,
            }),
            reserved104: [48]u8,
            ///  TIM timer input selection register
            TISEL: mmio.Mmio(packed struct(u32) {
                ///  TI1[0] to TI1[15] input selection
                TISEL: u4,
                padding: u28,
            }),
        };

        ///  General-purpose-timers
        pub const TIM2 = extern struct {
            ///  control register 1
            CR1: mmio.Mmio(packed struct(u32) {
                ///  Counter enable
                CEN: u1,
                ///  Update disable
                UDIS: u1,
                ///  Update request source
                URS: u1,
                ///  One-pulse mode
                OPM: u1,
                ///  Direction
                DIR: u1,
                ///  Center-aligned mode selection
                CMS: u2,
                ///  Auto-reload preload enable
                ARPE: u1,
                ///  Clock division
                CKD: u2,
                reserved11: u1,
                ///  UIF status bit remapping
                UIFREMAP: u1,
                padding: u20,
            }),
            ///  control register 2
            CR2: mmio.Mmio(packed struct(u32) {
                reserved3: u3,
                ///  Capture/compare DMA selection
                CCDS: u1,
                ///  Master mode selection
                MMS: u3,
                ///  TI1 selection
                TI1S: u1,
                padding: u24,
            }),
            ///  slave mode control register
            SMCR: mmio.Mmio(packed struct(u32) {
                ///  Slave mode selection
                SMS: u3,
                ///  OCREF clear selection
                OCCS: u1,
                ///  Trigger selection
                TS: u3,
                ///  Master/Slave mode
                MSM: u1,
                ///  External trigger filter
                ETF: u4,
                ///  External trigger prescaler
                ETPS: u2,
                ///  External clock enable
                ECE: u1,
                ///  External trigger polarity
                ETP: u1,
                ///  Slave mode selection - bit 3
                SMS_3: u1,
                reserved20: u3,
                ///  Trigger selection
                TS_4_3: u2,
                padding: u10,
            }),
            ///  DMA/Interrupt enable register
            DIER: mmio.Mmio(packed struct(u32) {
                ///  Update interrupt enable
                UIE: u1,
                ///  Capture/Compare 1 interrupt enable
                CC1IE: u1,
                ///  Capture/Compare 2 interrupt enable
                CC2IE: u1,
                ///  Capture/Compare 3 interrupt enable
                CC3IE: u1,
                ///  Capture/Compare 4 interrupt enable
                CC4IE: u1,
                reserved6: u1,
                ///  Trigger interrupt enable
                TIE: u1,
                reserved8: u1,
                ///  Update DMA request enable
                UDE: u1,
                ///  Capture/Compare 1 DMA request enable
                CC1DE: u1,
                ///  Capture/Compare 2 DMA request enable
                CC2DE: u1,
                ///  Capture/Compare 3 DMA request enable
                CC3DE: u1,
                ///  Capture/Compare 4 DMA request enable
                CC4DE: u1,
                reserved14: u1,
                ///  Trigger DMA request enable
                TDE: u1,
                padding: u17,
            }),
            ///  status register
            SR: mmio.Mmio(packed struct(u32) {
                ///  Update interrupt flag
                UIF: u1,
                ///  Capture/compare 1 interrupt flag
                CC1IF: u1,
                ///  Capture/Compare 2 interrupt flag
                CC2IF: u1,
                ///  Capture/Compare 3 interrupt flag
                CC3IF: u1,
                ///  Capture/Compare 4 interrupt flag
                CC4IF: u1,
                reserved6: u1,
                ///  Trigger interrupt flag
                TIF: u1,
                reserved9: u2,
                ///  Capture/Compare 1 overcapture flag
                CC1OF: u1,
                ///  Capture/compare 2 overcapture flag
                CC2OF: u1,
                ///  Capture/Compare 3 overcapture flag
                CC3OF: u1,
                ///  Capture/Compare 4 overcapture flag
                CC4OF: u1,
                padding: u19,
            }),
            ///  event generation register
            EGR: mmio.Mmio(packed struct(u32) {
                ///  Update generation
                UG: u1,
                ///  Capture/compare 1 generation
                CC1G: u1,
                ///  Capture/compare 2 generation
                CC2G: u1,
                ///  Capture/compare 3 generation
                CC3G: u1,
                ///  Capture/compare 4 generation
                CC4G: u1,
                reserved6: u1,
                ///  Trigger generation
                TG: u1,
                padding: u25,
            }),
            ///  capture/compare mode register 1 (output mode)
            CCMR1_Output: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 1 selection
                CC1S: u2,
                ///  Output compare 1 fast enable
                OC1FE: u1,
                ///  Output compare 1 preload enable
                OC1PE: u1,
                ///  Output compare 1 mode
                OC1M: u3,
                ///  Output compare 1 clear enable
                OC1CE: u1,
                ///  Capture/Compare 2 selection
                CC2S: u2,
                ///  Output compare 2 fast enable
                OC2FE: u1,
                ///  Output compare 2 preload enable
                OC2PE: u1,
                ///  Output compare 2 mode
                OC2M: u3,
                ///  Output compare 2 clear enable
                OC2CE: u1,
                ///  Output Compare 1 mode - bit 3
                OC1M_3: u1,
                reserved24: u7,
                ///  Output Compare 2 mode - bit 3
                OC2M_3: u1,
                padding: u7,
            }),
            ///  capture/compare mode register 2 (output mode)
            CCMR2_Output: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 3 selection
                CC3S: u2,
                ///  Output compare 3 fast enable
                OC3FE: u1,
                ///  Output compare 3 preload enable
                OC3PE: u1,
                ///  Output compare 3 mode
                OC3M: u3,
                ///  Output compare 3 clear enable
                OC3CE: u1,
                ///  Capture/Compare 4 selection
                CC4S: u2,
                ///  Output compare 4 fast enable
                OC4FE: u1,
                ///  Output compare 4 preload enable
                OC4PE: u1,
                ///  Output compare 4 mode
                OC4M: u3,
                ///  Output compare 4 clear enable
                OC4CE: u1,
                ///  Output Compare 3 mode - bit 3
                OC3M_3: u1,
                reserved24: u7,
                ///  Output Compare 4 mode - bit 3
                OC4M_3: u1,
                padding: u7,
            }),
            ///  capture/compare enable register
            CCER: mmio.Mmio(packed struct(u32) {
                ///  Capture/Compare 1 output enable
                CC1E: u1,
                ///  Capture/Compare 1 output Polarity
                CC1P: u1,
                reserved3: u1,
                ///  Capture/Compare 1 output Polarity
                CC1NP: u1,
                ///  Capture/Compare 2 output enable
                CC2E: u1,
                ///  Capture/Compare 2 output Polarity
                CC2P: u1,
                reserved7: u1,
                ///  Capture/Compare 2 output Polarity
                CC2NP: u1,
                ///  Capture/Compare 3 output enable
                CC3E: u1,
                ///  Capture/Compare 3 output Polarity
                CC3P: u1,
                reserved11: u1,
                ///  Capture/Compare 3 output Polarity
                CC3NP: u1,
                ///  Capture/Compare 4 output enable
                CC4E: u1,
                ///  Capture/Compare 3 output Polarity
                CC4P: u1,
                reserved15: u1,
                ///  Capture/Compare 4 output Polarity
                CC4NP: u1,
                padding: u16,
            }),
            ///  counter
            CNT: mmio.Mmio(packed struct(u32) {
                ///  Low counter value
                CNT_L: u16,
                ///  High counter value (TIM2 only)
                CNT_H: u16,
            }),
            ///  prescaler
            PSC: mmio.Mmio(packed struct(u32) {
                ///  Prescaler value
                PSC: u16,
                padding: u16,
            }),
            ///  auto-reload register
            ARR: mmio.Mmio(packed struct(u32) {
                ///  Low Auto-reload value
                ARR_L: u16,
                ///  High Auto-reload value (TIM2 only)
                ARR_H: u16,
            }),
            reserved52: [4]u8,
            ///  capture/compare register 1
            CCR1: mmio.Mmio(packed struct(u32) {
                ///  Low Capture/Compare 1 value
                CCR1_L: u16,
                ///  High Capture/Compare 1 value (TIM2 only)
                CCR1_H: u16,
            }),
            ///  capture/compare register 2
            CCR2: mmio.Mmio(packed struct(u32) {
                ///  Low Capture/Compare 2 value
                CCR2_L: u16,
                ///  High Capture/Compare 2 value (TIM2 only)
                CCR2_H: u16,
            }),
            ///  capture/compare register 3
            CCR3: mmio.Mmio(packed struct(u32) {
                ///  Low Capture/Compare value
                CCR3_L: u16,
                ///  High Capture/Compare value (TIM2 only)
                CCR3_H: u16,
            }),
            ///  capture/compare register 4
            CCR4: mmio.Mmio(packed struct(u32) {
                ///  Low Capture/Compare value
                CCR4_L: u16,
                ///  High Capture/Compare value (TIM2 only)
                CCR4_H: u16,
            }),
            reserved72: [4]u8,
            ///  DMA control register
            DCR: mmio.Mmio(packed struct(u32) {
                ///  DMA base address
                DBA: u5,
                reserved8: u3,
                ///  DMA burst length
                DBL: u5,
                padding: u19,
            }),
            ///  DMA address for full transfer
            DMAR: mmio.Mmio(packed struct(u32) {
                ///  DMA register for burst accesses
                DMAB: u16,
                padding: u16,
            }),
            ///  TIM option register
            OR1: mmio.Mmio(packed struct(u32) {
                ///  IOCREF_CLR
                IOCREF_CLR: u1,
                padding: u31,
            }),
            reserved96: [12]u8,
            ///  TIM alternate function option register 1
            AF1: mmio.Mmio(packed struct(u32) {
                reserved14: u14,
                ///  External trigger source selection
                ETRSEL: u4,
                padding: u14,
            }),
            reserved104: [4]u8,
            ///  TIM alternate function option register 1
            TISEL: mmio.Mmio(packed struct(u32) {
                ///  TI1SEL
                TI1SEL: u4,
                reserved8: u4,
                ///  TI2SEL
                TI2SEL: u4,
                padding: u20,
            }),
        };

        ///  System control block
        pub const SCB = extern struct {
            ///  CPUID base register
            CPUID: mmio.Mmio(packed struct(u32) {
                ///  Revision number
                Revision: u4,
                ///  Part number of the processor
                PartNo: u12,
                ///  Reads as 0xF
                Architecture: u4,
                ///  Variant number
                Variant: u4,
                ///  Implementer code
                Implementer: u8,
            }),
            ///  Interrupt control and state register
            ICSR: mmio.Mmio(packed struct(u32) {
                ///  Active vector
                VECTACTIVE: u9,
                reserved11: u2,
                ///  Return to base level
                RETTOBASE: u1,
                ///  Pending vector
                VECTPENDING: u7,
                reserved22: u3,
                ///  Interrupt pending flag
                ISRPENDING: u1,
                reserved25: u2,
                ///  SysTick exception clear-pending bit
                PENDSTCLR: u1,
                ///  SysTick exception set-pending bit
                PENDSTSET: u1,
                ///  PendSV clear-pending bit
                PENDSVCLR: u1,
                ///  PendSV set-pending bit
                PENDSVSET: u1,
                reserved31: u2,
                ///  NMI set-pending bit.
                NMIPENDSET: u1,
            }),
            ///  Vector table offset register
            VTOR: mmio.Mmio(packed struct(u32) {
                reserved7: u7,
                ///  Vector table base offset field
                TBLOFF: u25,
            }),
            ///  Application interrupt and reset control register
            AIRCR: mmio.Mmio(packed struct(u32) {
                reserved1: u1,
                ///  VECTCLRACTIVE
                VECTCLRACTIVE: u1,
                ///  SYSRESETREQ
                SYSRESETREQ: u1,
                reserved15: u12,
                ///  ENDIANESS
                ENDIANESS: u1,
                ///  Register key
                VECTKEYSTAT: u16,
            }),
            ///  System control register
            SCR: mmio.Mmio(packed struct(u32) {
                reserved1: u1,
                ///  SLEEPONEXIT
                SLEEPONEXIT: u1,
                ///  SLEEPDEEP
                SLEEPDEEP: u1,
                reserved4: u1,
                ///  Send Event on Pending bit
                SEVEONPEND: u1,
                padding: u27,
            }),
            ///  Configuration and control register
            CCR: mmio.Mmio(packed struct(u32) {
                ///  Configures how the processor enters Thread mode
                NONBASETHRDENA: u1,
                ///  USERSETMPEND
                USERSETMPEND: u1,
                reserved3: u1,
                ///  UNALIGN_ TRP
                UNALIGN__TRP: u1,
                ///  DIV_0_TRP
                DIV_0_TRP: u1,
                reserved8: u3,
                ///  BFHFNMIGN
                BFHFNMIGN: u1,
                ///  STKALIGN
                STKALIGN: u1,
                padding: u22,
            }),
            reserved28: [4]u8,
            ///  System handler priority registers
            SHPR2: mmio.Mmio(packed struct(u32) {
                reserved24: u24,
                ///  Priority of system handler 11
                PRI_11: u8,
            }),
            ///  System handler priority registers
            SHPR3: mmio.Mmio(packed struct(u32) {
                reserved16: u16,
                ///  Priority of system handler 14
                PRI_14: u8,
                ///  Priority of system handler 15
                PRI_15: u8,
            }),
        };

        ///  Nested Vectored Interrupt Controller
        pub const NVIC = extern struct {
            ///  Interrupt Set Enable Register
            ISER: mmio.Mmio(packed struct(u32) {
                ///  SETENA
                SETENA: u32,
            }),
            reserved128: [124]u8,
            ///  Interrupt Clear Enable Register
            ICER: mmio.Mmio(packed struct(u32) {
                ///  CLRENA
                CLRENA: u32,
            }),
            reserved256: [124]u8,
            ///  Interrupt Set-Pending Register
            ISPR: mmio.Mmio(packed struct(u32) {
                ///  SETPEND
                SETPEND: u32,
            }),
            reserved384: [124]u8,
            ///  Interrupt Clear-Pending Register
            ICPR: mmio.Mmio(packed struct(u32) {
                ///  CLRPEND
                CLRPEND: u32,
            }),
            reserved768: [380]u8,
            ///  Interrupt Priority Register 0
            IPR0: mmio.Mmio(packed struct(u32) {
                ///  priority for interrupt 0
                PRI_0: u8,
                ///  priority for interrupt 1
                PRI_1: u8,
                ///  priority for interrupt 2
                PRI_2: u8,
                ///  priority for interrupt 3
                PRI_3: u8,
            }),
            ///  Interrupt Priority Register 1
            IPR1: mmio.Mmio(packed struct(u32) {
                ///  priority for interrupt n
                PRI_4: u8,
                ///  priority for interrupt n
                PRI_5: u8,
                ///  priority for interrupt n
                PRI_6: u8,
                ///  priority for interrupt n
                PRI_7: u8,
            }),
            ///  Interrupt Priority Register 2
            IPR2: mmio.Mmio(packed struct(u32) {
                ///  priority for interrupt n
                PRI_8: u8,
                ///  priority for interrupt n
                PRI_9: u8,
                ///  priority for interrupt n
                PRI_10: u8,
                ///  priority for interrupt n
                PRI_11: u8,
            }),
            ///  Interrupt Priority Register 3
            IPR3: mmio.Mmio(packed struct(u32) {
                ///  priority for interrupt n
                PRI_12: u8,
                ///  priority for interrupt n
                PRI_13: u8,
                ///  priority for interrupt n
                PRI_14: u8,
                ///  priority for interrupt n
                PRI_15: u8,
            }),
            ///  Interrupt Priority Register 4
            IPR4: mmio.Mmio(packed struct(u32) {
                ///  priority for interrupt n
                PRI_16: u8,
                ///  priority for interrupt n
                PRI_17: u8,
                ///  priority for interrupt n
                PRI_18: u8,
                ///  priority for interrupt n
                PRI_19: u8,
            }),
            ///  Interrupt Priority Register 5
            IPR5: mmio.Mmio(packed struct(u32) {
                ///  priority for interrupt n
                PRI_20: u8,
                ///  priority for interrupt n
                PRI_21: u8,
                ///  priority for interrupt n
                PRI_22: u8,
                ///  priority for interrupt n
                PRI_23: u8,
            }),
            ///  Interrupt Priority Register 6
            IPR6: mmio.Mmio(packed struct(u32) {
                ///  priority for interrupt n
                PRI_24: u8,
                ///  priority for interrupt n
                PRI_25: u8,
                ///  priority for interrupt n
                PRI_26: u8,
                ///  priority for interrupt n
                PRI_27: u8,
            }),
            ///  Interrupt Priority Register 7
            IPR7: mmio.Mmio(packed struct(u32) {
                ///  priority for interrupt n
                PRI_28: u8,
                ///  priority for interrupt n
                PRI_29: u8,
                ///  priority for interrupt n
                PRI_30: u8,
                ///  priority for interrupt n
                PRI_31: u8,
            }),
            ///  Interrupt Priority Register 8
            IPR8: u32,
        };

        ///  Memory protection unit
        pub const MPU = extern struct {
            ///  MPU type register
            MPU_TYPER: mmio.Mmio(packed struct(u32) {
                ///  Separate flag
                SEPARATE: u1,
                reserved8: u7,
                ///  Number of MPU data regions
                DREGION: u8,
                ///  Number of MPU instruction regions
                IREGION: u8,
                padding: u8,
            }),
            ///  MPU control register
            MPU_CTRL: mmio.Mmio(packed struct(u32) {
                ///  Enables the MPU
                ENABLE: u1,
                ///  Enables the operation of MPU during hard fault
                HFNMIENA: u1,
                ///  Enable priviliged software access to default memory map
                PRIVDEFENA: u1,
                padding: u29,
            }),
            ///  MPU region number register
            MPU_RNR: mmio.Mmio(packed struct(u32) {
                ///  MPU region
                REGION: u8,
                padding: u24,
            }),
            ///  MPU region base address register
            MPU_RBAR: mmio.Mmio(packed struct(u32) {
                ///  MPU region field
                REGION: u4,
                ///  MPU region number valid
                VALID: u1,
                ///  Region base address field
                ADDR: u27,
            }),
            ///  MPU region attribute and size register
            MPU_RASR: mmio.Mmio(packed struct(u32) {
                ///  Region enable bit.
                ENABLE: u1,
                ///  Size of the MPU protection region
                SIZE: u5,
                reserved8: u2,
                ///  Subregion disable bits
                SRD: u8,
                ///  memory attribute
                B: u1,
                ///  memory attribute
                C: u1,
                ///  Shareable memory attribute
                S: u1,
                ///  memory attribute
                TEX: u3,
                reserved24: u2,
                ///  Access permission
                AP: u3,
                reserved28: u1,
                ///  Instruction access disable bit
                XN: u1,
                padding: u3,
            }),
        };

        ///  SysTick timer
        pub const STK = extern struct {
            ///  SysTick control and status register
            CSR: mmio.Mmio(packed struct(u32) {
                ///  Counter enable
                ENABLE: u1,
                ///  SysTick exception request enable
                TICKINT: u1,
                ///  Clock source selection
                CLKSOURCE: u1,
                reserved16: u13,
                ///  COUNTFLAG
                COUNTFLAG: u1,
                padding: u15,
            }),
            ///  SysTick reload value register
            RVR: mmio.Mmio(packed struct(u32) {
                ///  RELOAD value
                RELOAD: u24,
                padding: u8,
            }),
            ///  SysTick current value register
            CVR: mmio.Mmio(packed struct(u32) {
                ///  Current counter value
                CURRENT: u24,
                padding: u8,
            }),
            ///  SysTick calibration value register
            CALIB: mmio.Mmio(packed struct(u32) {
                ///  Calibration value
                TENMS: u24,
                reserved30: u6,
                ///  SKEW flag: Indicates whether the TENMS value is exact
                SKEW: u1,
                ///  NOREF flag. Reads as zero
                NOREF: u1,
            }),
        };
    };
};
