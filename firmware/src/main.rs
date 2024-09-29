//#![deny(warnings)]
#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

mod qei;

#[rtic::app(device = stm32g4xx_hal::stm32, peripherals = true)]
mod app{
    use super::*;

    use core::fmt::Write;
    use embedded_hal::adc::{self, Channel};
    use qei::Qei;
    use stm32g4xx_hal::dma::config::DmaConfig;
    use stm32g4xx_hal::dma::stream::{DMAExt, Stream0};
    use stm32g4xx_hal::dma::transfer::{self, MutTransfer};
    use stm32g4xx_hal::dma::{PeripheralToMemory, Transfer, TransferExt};
    use stm32g4xx_hal::gpio::{Alternate, AlternateOD, Floating, Input, Output, PullUp, PushPull, AF2, AF8};
    use stm32g4xx_hal::i2c::{self, I2c};
    use stm32g4xx_hal::rcc::PllConfig;
    use stm32g4xx_hal::rcc::PllMDiv;
    use stm32g4xx_hal::rcc::PllNMul;
    use stm32g4xx_hal::rcc::PllRDiv;
    use stm32g4xx_hal::stm32::{ADC1, DMA1, I2C3, TIM3};
    use stm32g4xx_hal as hal;
    use hal::prelude::*;
    use hal::time::{RateExtU32, ExtU32};
    use stm32g4xx_hal::adc::config::ExternalTrigger12;
    use stm32g4xx_hal::adc::config::TriggerMode;
    use stm32g4xx_hal::gpio::ExtiPin;
    use stm32g4xx_hal::gpio::gpioa::{PA, PA10, PA7, PA8};
    use stm32g4xx_hal::gpio::gpiob::{PB4, PB5};
    use stm32g4xx_hal::gpio::gpiof::{PF0, PF1};
    use stm32g4xx_hal::pwr::PwrExt;
    use stm32g4xx_hal::syscfg::SysCfgExt;
    use stm32g4xx_hal::delay::{Delay, DelayFromCountDownTimer};
    use stm32g4xx_hal::timer::{Timer,CountDownTimer};
    use stm32g4xx_hal::rcc::Config;
    use stm32g4xx_hal::adc::{Active, Adc, AdcClaim, ClockSource, Configured, Temperature, DMA};
    use stm32g4xx_hal::adc::config::{Continuous, Eoc, Sequence, SampleTime};

    use sh1106::{prelude::*, Builder};
    use embedded_graphics::prelude::*;
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::mono_font::{
        ascii::FONT_6X12,
        MonoTextStyleBuilder
    };
    use embedded_graphics::text::Text;
    use heapless::String;

    use cortex_m_rt::entry;
    use cortex_m::interrupt::free;
    use cortex_m::interrupt::Mutex;

    use defmt::{info, error};


    #[shared]
    struct Shared {

    }

    #[local]
    struct Local {
        led1: PF0<Output<PushPull>>,
        led2: PF1<Output<PushPull>>,
        on_off_sw: PA10<Input<PullUp>>,
        transfer: Transfer<Stream0<DMA1>, Adc<ADC1, DMA>, PeripheralToMemory, &'static mut [u16], MutTransfer>,
        disp: GraphicsMode<I2cInterface<I2c<I2C3, PB5<AlternateOD<AF8>>, PA8<AlternateOD<AF2>>>>>,
        motor_enc: Qei<TIM3, PB4<Alternate<AF2>>, PA7<Alternate<AF2>>>,
    }

/*

A0 - ISNS, 20x 0.015 A/V
A1 - VSNS 10 / 230
A2 - ENC_A
A3 - ENC_B
A4 - ENC_BTN
A5 - SPEED
A6 - STEP
A7 - PHASE_B
A8 - DISP_CLK (I2C3)
A9 - DIR
A10 - ON_SW
A11 - USB_N
A12 - USB_P
A13 - SWDIO/BTN_ENT
A14 - SWCLK
A15 - ENABLE
B0 - SWB_LS 8.2N
B3 - SWA_LS 8.1N
B4 - PHASE_A
B5 - DISP_DAT (I2C3)
B6 - SWA_HS 8.1
B7 - OV/FAN
B8 - SWB_HS 8.2
*/

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dp = ctx.device;
        dp.RCC.apb1enr1.write(|w| w.pwren().set_bit());
        dp.PWR.cr3.write(|w| w.ucpd1_dbdis().set_bit());

        let pwr = dp.PWR.constrain().freeze();
        let rcc = dp.RCC.constrain();
        /* PLL Configured for 128MHz from 16MHz HSI */
        let mut pll_cfg = PllConfig::default();
        pll_cfg.m = PllMDiv::DIV_2;
        pll_cfg.n = PllNMul::MUL_32;
        pll_cfg.r = Some(PllRDiv::DIV_2);
        let mut rcc = rcc.freeze(Config::pll().pll_cfg(pll_cfg), pwr);

        let mut syscfg = dp.SYSCFG.constrain();

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpiof = dp.GPIOF.split(&mut rcc);

        let mut low_side_a = gpiob.pb3.into_push_pull_output();
        low_side_a.set_low().unwrap();
        let mut low_side_b = gpiob.pb0.into_push_pull_output();
        low_side_b.set_high().unwrap();
        let mut fan_en = gpiob.pb7.into_push_pull_output();
        fan_en.set_low().unwrap();

        let mut led1 = gpiof.pf0.into_push_pull_output();
        led1.set_low().unwrap();

        let mut led2 = gpiof.pf1.into_push_pull_output();
        led2.set_low().unwrap();

        let mut motor_on = gpioa.pa10.into_pull_up_input();
        motor_on.make_interrupt_source(&mut syscfg);
        motor_on.enable_interrupt(&mut dp.EXTI);
        motor_on.trigger_on_edge(&mut dp.EXTI, stm32g4xx_hal::gpio::SignalEdge::RisingFalling);

        let high_side_a = gpiob.pb6.into_alternate();
        let high_side_b = gpiob.pb8.into_alternate();
        let (mut pwm_a, mut pwm_b) = dp.TIM8.pwm((high_side_a, high_side_b), 30.kHz(), &mut rcc);

        // Enable UEV as trigger
        unsafe { (*hal::stm32::TIM8::ptr()).cr2.write(|w| w.mms().bits(2)) };

        pwm_a.set_duty(pwm_a.get_max_duty() / 7);
        pwm_a.enable();
        pwm_b.set_duty(0);
        pwm_b.enable();

        let enc_a: PB4<Alternate<AF2>> = gpiob.pb4.into_alternate(); // AF2 TIM3.1
        let enc_b: PA7<Alternate<AF2>> = gpioa.pa7.into_alternate(); // AF2 TIM3.2
        //unsafe { (*stm32::GPIOB::ptr()).pupdr.write(|w| w.pupdr4().floating()); };
        let motor_enc = qei::Qei::new(dp.TIM3, enc_a, enc_b);


        let scl = gpioa.pa8.into_alternate_open_drain();
        let sda = gpiob.pb5.into_alternate_open_drain();
        let i2c = dp.I2C3.i2c(sda, scl, i2c::Config::new(10.kHz()), &mut rcc);
        let mut disp: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();
        disp.init().unwrap();
        disp.flush().unwrap();


        let timer = Timer::new(dp.TIM1, &rcc.clocks).start_count_down(100.millis());
        let mut delay = DelayFromCountDownTimer::new(timer);
        let isns = gpioa.pa0.into_analog();
        let vsns = gpioa.pa1.into_analog();
        let speed = gpioa.pa5.into_analog();
        let mut adc = dp.ADC1.claim(ClockSource::SystemClock, &mut rcc, &mut delay, true);
        adc.enable_temperature(&dp.ADC12_COMMON);
        adc.set_auto_delay(true);
        adc.set_continuous(Continuous::Single);
        adc.reset_sequence();
        //adc.configure_channel(&isns, Sequence::One, SampleTime::Cycles_640_5);
        //adc.configure_channel(&vsns, Sequence::Two, SampleTime::Cycles_92_5);
        //adc.configure_channel(&Temperature, Sequence::Three, SampleTime::Cycles_92_5);
        //adc.set_end_of_conversion_interrupt(Eoc::Sequence);
        adc.set_external_trigger((TriggerMode::RisingEdge, ExternalTrigger12::Tim_8_trgo));

        let mut streams = dp.DMA1.split(&rcc);
        let config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .circular_buffer(true)
            .memory_increment(true);
        let adc_buffer = cortex_m::singleton!(: [u16; 3] = [0; 3]).unwrap();
        let mut transfer = streams.0.into_peripheral_to_memory_transfer(
            adc.enable_dma(hal::adc::config::Dma::Continuous),
            &mut adc_buffer[..],
            config
        );

        transfer.start(|_adc| {});

        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::EXTI15_10);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::DMA1_CH1);
        }

        info!("starting main loop.");
        info!("sysclk = {}", rcc.clocks.sys_clk.to_MHz());

        (
            Shared {},
            Local {
                led1,
                led2,
                on_off_sw: motor_on,
                disp,
                transfer,
                motor_enc,
            }
        )
    }


    #[idle(local = [motor_enc, led2, disp])]
    fn idle(mut ctx: idle::Context) -> ! {
        //let adc = ctx.local.adc;
        //let mut adc = adc.start_conversion();
        let enc = ctx.local.motor_enc;
        let disp = ctx.local.disp;
        let led2 = ctx.local.led2;
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X12)
            .text_color(BinaryColor::On)
            .build();
        disp.flush().unwrap();

        let mut i = 0;
        loop {
            //let adc = adc.wait_for_conversion_sequence().unwrap_active();
            //let mv = adc.sample_to_millivolts(adc.current_sample());
            let mv = 0;
            disp.clear();
            let mut dis_str: String<20> = String::new();
            write!(dis_str, "Hello: {}", mv).unwrap();
            Text::new(&dis_str, Point::new(0,10), style)
            .draw(disp)
            .unwrap();
            let mut enc_str: String<20> = String::new();
            write!(enc_str, "ENC: {}", enc.count()).unwrap();
            Text::new(&enc_str, Point::new(0,30), style)
            .draw(disp)
            .unwrap();
            disp.flush().unwrap();

            led2.set_low().unwrap();
            //delay.delay_ms(100);

            led2.set_high().unwrap();
            //delay.delay_ms(100);

            i += 1;
        }
    }

    #[task(binds = EXTI15_10, local = [led1, on_off_sw])]
    fn gpio_interrupt_handler(mut ctx: gpio_interrupt_handler::Context) {
        let sw = ctx.local.on_off_sw;
        sw.clear_interrupt_pending_bit();

        ctx.local.led1.toggle().unwrap();

    }

    #[task(binds = DMA1_CH1, local = [transfer])]
    fn dma_interrupt(mut ctx: dma_interrupt::Context) {
        let transfer = ctx.local.transfer;
        transfer.clear_interrupts();

        info!("dma");
    }

}
