//#![deny(warnings)]
#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

mod qei;
mod adc;
mod controller;
mod motor;

#[rtic::app(device = stm32g4xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app{
    use super::*;

    use core::fmt::Write;
    use cortex_m::register::control;
    use qei::Qei;
    use rtic_monotonics::systick_monotonic;
    use rtic_sync::channel::Receiver;
    use stm32g4xx_hal as hal;
    use hal::prelude::*;
    use hal::time::{RateExtU32, ExtU32};
    use stm32g4xx_hal::gpio::{Alternate, AlternateOD, Input, Output, PullUp, PushPull, AF14, AF15, AF2, AF8};
    use stm32g4xx_hal::i2c::{self, I2c};
    use stm32g4xx_hal::rcc::{Config, PllConfig, PllMDiv, PllNMul, PllRDiv};
    use stm32g4xx_hal::stm32::{I2C3, TIM3};
    use stm32g4xx_hal::gpio::gpioa::{PA11, PA12, PA10, PA7, PA8};
    use stm32g4xx_hal::gpio::gpiob::{PB4, PB5};
    use stm32g4xx_hal::gpio::gpiof::{PF0, PF1};
    use stm32g4xx_hal::pwr::PwrExt;
    use stm32g4xx_hal::usb::{Peripheral, UsbBus};

    use usb_device::bus::UsbBusAllocator;
    use usb_device::device::UsbDevice;
    use usb_device::prelude::*;
    use usbd_serial::{SerialPort, USB_CLASS_CDC};
    use rotary_encoder_hal::{Direction, Rotary};
    use sh1106::{prelude::*, Builder};
    use embedded_graphics::prelude::*;
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::mono_font::{
        ascii::FONT_6X12,
        MonoTextStyleBuilder
    };
    use rtic_sync::{channel::*, make_channel};
    use rtic_monotonics::systick::prelude::*;
    use embedded_graphics::text::Text;
    use heapless::{String, Vec};

    use defmt::{info, error};

    type UsbBusType = UsbBus<Peripheral<PA11<Alternate<AF14>>, PA12<Alternate<AF14>>>>;
    systick_monotonic!(Mono, 1_000);

    #[shared]
    struct Shared {
        control: controller::Controller,
        usb_serial: SerialPort<'static, UsbBus<Peripheral<PA11<Alternate<AF14>>, PA12<Alternate<AF14>>>>>,
    }

    const QUEUE_DEPTH: usize = 8;
    #[local]
    struct Local {
        adc1: adc::Adc1,
        adc2: adc::Adc2,
        led1: PF0<Output<PushPull>>,
        led2: PF1<Output<PushPull>>,
        motor: motor::Motor,
        motor_on: PA10<Input<PullUp>>,
        disp: GraphicsMode<I2cInterface<I2c<I2C3, PB5<AlternateOD<AF8>>, PA8<AlternateOD<AF2>>>>>,
        motor_enc: Qei<TIM3, PB4<Alternate<AF2>>, PA7<Alternate<AF2>>>,
        usb_dev: UsbDevice<'static, UsbBus<Peripheral<PA11<Alternate<AF14>>, PA12<Alternate<AF14>>>>>,
        msg_tx: Sender<'static, Vec<u8, 16>, QUEUE_DEPTH>,
        msg_rx: Receiver<'static, Vec<u8, 16>, QUEUE_DEPTH>,
    }

/*
TIMERS:
 - TIM2 -> Pos Enc Rate
 - TIM3 -> Pos Enc Count
 - TIM8 -> Torque PWM @30KHz
 - TIM15 -> 1KHz Speed Loop Tick
 - TIM16 -> STEP Gen

I/O:
    A0 - ISNS       (ADC1.AIN1)
    A1 - VSNS       (ADC2.AIN2)
    A2 - ENC_A      
    A3 - ENC_B      
    A4 - ENC_BTN
    A5 - SPEED      (ADC2.AIN14)
    A6 - STEP       (TIM16.1, AF1)
    A7 - PHASE_B    (TIM3.2, AF2)
    A8 - DISP_CLK   (I2C3)
    A9 - DIR
    A10 - ON_SW
    A11 - USB_N     (USB)
    A12 - USB_P     (USB)
    A13 - SWDIO/BTN_ENT
    A14 - SWCLK
    A15 - ENABLE
    B0 - SWB_LS     (TIM8.2N
    B3 - SWA_LS     (TIM8.1N
    B4 - PHASE_A    (TIM3.1, AF2)
    B5 - DISP_DAT   (I2C3)
    B6 - SWA_HS     (TIM8.1)
    B7 - OV/FAN
    B8 - SWB_HS     (TIM8.2)

Interrupts / Tasks:
    - ADC1_2() -> 30KHz Torque Control, triggered from TIM8
        - Owns: ADC 1, PWM_FW, PWM_BACK, FW/REV GPIO
        - Shares: Cntrl
    - TIM1_BRK_TIM15() -> 1KHz Speed Control
        - Owns TIM15, TIM2, TIM3
        - Sares: Cntrl, ADC2?
    - USB? 
        - Owns: USB
    - User Menu Tick: Poll Enc/Btn, update display
        - Owns: Disp, ENC, BTN, LEDs
        - Shares: StateMachine
    - ADC Background Task: Update Speed/Voltage/Temp
        - Owns: ADC2?
        - Shares: Cntl, StateMachine
    - Future Position Control Task (100Hz) 
    - Future Step Drive Task (100Hz?)
*/

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBusType>> = None])]
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
        rcc.enable_hsi48();

        Mono::start(ctx.core.SYST, rcc.clocks.sys_clk.to_Hz());

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpiof = dp.GPIOF.split(&mut rcc);

        // ADC, must happen before PWM is enabled
        let _isns = gpioa.pa0.into_analog();
        let _vsns = gpioa.pa1.into_analog();
        let _speed = gpioa.pa5.into_analog();
        let adc1 = adc::Adc1::new();
        let adc2 = adc::Adc2::new();

        // Motor PWM GPIO
        let mut low_side_a = gpiob.pb3.into_push_pull_output();
        let mut low_side_b = gpiob.pb0.into_push_pull_output();
        low_side_a.set_low().unwrap();
        low_side_b.set_high().unwrap();
        let high_side_a = gpiob.pb6.into_alternate();
        let high_side_b = gpiob.pb8.into_alternate();
        let (mut pwm_a, mut pwm_b) = dp.TIM8.pwm((high_side_a, high_side_b), 30.kHz(), &mut rcc);
        // Enable UEV as trigger
        unsafe { (*hal::stm32::TIM8::ptr()).cr2.write(|w| w.mms2().bits(2)) };
        pwm_a.set_duty(0);
        pwm_a.enable();
        pwm_b.set_duty(0);
        pwm_b.enable();

        let mut motor = motor::Motor {
            pwm_fwd: pwm_a,
            pwm_rev: pwm_b,
            fwd_en: low_side_b,
            rev_en: low_side_a,
            forward: true,
        };

        // LED
        let mut led1 = gpiof.pf0.into_push_pull_output();
        let mut led2 = gpiof.pf1.into_push_pull_output();
        led1.set_low().unwrap();
        led2.set_low().unwrap();

        // GPIO
        let mut motor_on = gpioa.pa10.into_pull_up_input();
        let mut fan_en = gpiob.pb7.into_push_pull_output();
        fan_en.set_low().unwrap();

        // USB Serial
        let usb_dm = gpioa.pa11.into_alternate();
        let usb_dp = gpioa.pa12.into_alternate();

        let usb = Peripheral {
            usb: dp.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };
        let usb_bus: &'static _ = ctx.local.usb_bus.insert(UsbBus::new(usb));
        let mut usb_serial = SerialPort::new(&usb_bus);
        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")])
            .unwrap()
            .device_class(USB_CLASS_CDC)
            .build();

        // Position Encoder
        let enc_a: PB4<Alternate<AF2>> = gpiob.pb4.into_alternate(); // AF2 TIM3.1
        let enc_b: PA7<Alternate<AF2>> = gpioa.pa7.into_alternate(); // AF2 TIM3.2
        let motor_enc = qei::Qei::new(dp.TIM3, enc_a, enc_b);

        // Display Encoder
        let mut dea = gpioa.pa2.into_pull_up_input();
        let mut deb = gpioa.pa3.into_pull_up_input();
        let mut disp_enc_btn = gpioa.pa4.into_pull_up_input();
        let mut disp_enc = Rotary::new(dea, deb);

        // Display
        cortex_m::asm::delay(20000000);
        led2.set_high().unwrap();
        let scl = gpioa.pa8.into_alternate_open_drain();
        let sda = gpiob.pb5.into_alternate_open_drain();
        let i2c = dp.I2C3.i2c(sda, scl, i2c::Config::new(100.kHz()), &mut rcc);
        let mut disp: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();
        disp.init().unwrap(); // TODO: Handle no-display
        disp.flush().unwrap();

        let mut control = controller::Controller::new();
        control.torque.set_target(0.400);
        control.speed.set_target(200.0);

        let (s, r) = make_channel!(Vec<u8, 16>, QUEUE_DEPTH);

        // Enable Interrupts
        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM1_BRK_TIM15);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::ADC1_2);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::USB_LP);
        }

        //usb_task::spawn().unwrap();
        display_task::spawn().unwrap();

        info!("starting main loop.");

        (
            Shared {
                control,
                usb_serial,
            },
            Local {
                adc1,
                adc2,
                led1,
                led2,
                motor,
                motor_on,
                disp,
                motor_enc,
                usb_dev,
                msg_rx: r,
                msg_tx: s,
            }
        )
    }

    #[task(priority = 1, local = [adc2, motor_on, led2, disp], shared = [control])]
    async fn display_task(mut ctx: display_task::Context) {
        info!("starting disp loop.");
        let adc = ctx.local.adc2;
        let enable = ctx.local.motor_on;
        let mut control = ctx.shared.control;
        let disp = ctx.local.disp;
        let led2 = ctx.local.led2;
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X12)
            .text_color(BinaryColor::On)
            .build();
        disp.flush().unwrap();

        let mut i = 0;
        let mut disp_str: String<32> = String::new();

        loop {
            adc.start_conversion();
            disp.clear();
            let voltage = adc::Adc2::sample_to_input_voltage(adc.read_sample());

            adc.start_conversion();
            let potentiometer = adc::Adc2::sample_to_volts(adc.read_sample());

            disp_str.clear();
            write!(disp_str, "IN: {:2.2}V, {:.1}V", voltage, potentiometer).unwrap();
            Text::new(&disp_str, Point::new(0,10), style).draw(disp).unwrap();


            let mut amps = 0.0;
            let mut duty_cycle = 0.0;
            let mut speed = 0.0;
            let mut cmd = 0.0;

            led2.set_low().unwrap();
            control.lock(|control| {
                amps = control.torque.current;
                duty_cycle = control.torque.duty_cycle;
                speed = control.speed.rpm;
                cmd = control.speed.command_torque;

                control.speed.set_target(potentiometer * 300.0);
                control.torque.enabled = enable.is_high().unwrap();
            });
            led2.set_high().unwrap();

            disp_str.clear();
            write!(disp_str, "S: {:.1}r, {:1.3}A", speed, cmd).unwrap();
            Text::new(&disp_str, Point::new(0,40), style).draw(disp).unwrap();

            disp_str.clear();
            write!(disp_str, "I: {:1.3}A, {:2.1}%", amps, duty_cycle * 100.0).unwrap();
            Text::new(&disp_str, Point::new(0,25), style).draw(disp).unwrap();

            disp.flush().unwrap();
            Mono::delay(10.millis()).await;
            Mono::delay(90.millis()).await;

            i += 1;
        }
    }


    #[task(binds = TIM1_BRK_TIM15, priority=4, shared = [control])]
    fn timer_interrupt(mut ctx: timer_interrupt::Context) {
        controller::Speed::clear_timer();
        let mut ctrl = ctx.shared.control;
        ctrl.lock(|ctrl| {
            let torque = ctrl.speed.control_cycle();
            ctrl.torque.set_target(torque);
        });
    }

    #[task(binds = ADC1_2, priority=5, shared = [control], local = [motor, adc1, led1])]
    fn adc_interrupt(mut ctx: adc_interrupt::Context) {
        let led2 = ctx.local.led1;
        let motor = ctx.local.motor;
        let adc = ctx.local.adc1;
        let sample =adc.read_sample();

        let mut ctrl = ctx.shared.control;
        ctrl.lock(|ctrl| {
            let duty_cycle = ctrl.torque.control_cycle(sample);
            motor.set_duty_cycle(duty_cycle);
        });

        led2.toggle().unwrap();
    }

    #[task(binds=USB_LP, priority=3, local = [msg_rx, usb_dev], shared = [usb_serial])]
    fn usb_interrupt(mut ctx: usb_interrupt::Context) {
        let mut usb_dev = ctx.local.usb_dev;
        let mut serial = ctx.shared.usb_serial;

        serial.lock(|serial| {
            if usb_dev.poll(&mut [serial])  {
                //info!("USB");
            }
        });
    }


}
