//#![deny(warnings)]
#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

mod qei;
mod adc;
mod controller;
mod motor;
mod message;

#[rtic::app(device = stm32g4xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app{
    use super::*;

    use core::ascii::escape_default;
    use core::fmt;
    use message::Message;
    use rtic_monotonics::systick_monotonic;
    use rtic_sync::channel::Receiver;
    use stm32g4xx_hal::comparator::EnabledState;
    use stm32g4xx_hal::pwm::{FaultMonitor, Polarity, PwmAdvExt};
    use stm32g4xx_hal as hal;
    use hal::prelude::*;
    use hal::time::{RateExtU32, ExtU32};
    use stm32g4xx_hal::gpio::{Alternate, AlternateOD, ExtiPin, Input, Output, PullUp, PushPull, AF14, AF15, AF2, AF8};
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
    use usbd_serial::embedded_io::Write;
    use usbd_serial::{SerialPort, USB_CLASS_CDC};
    use rotary_encoder_hal::Rotary;
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

    use defmt::info;

    type UsbBusType = UsbBus<Peripheral<PA11<Alternate<AF14>>, PA12<Alternate<AF14>>>>;
    systick_monotonic!(Mono, 1_000);

    #[shared]
    struct Shared {
        control: controller::Controller,
        usb_serial: SerialPort<'static, UsbBus<Peripheral<PA11<Alternate<AF14>>, PA12<Alternate<AF14>>>>>,
    }

    const QUEUE_DEPTH: usize = 4;
    const TELEM_DEPTH: usize = 32;
    #[local]
    struct Local {
        adc1: adc::Adc1,
        adc2: adc::Adc2,
        led1: PF0<Output<PushPull>>,
        led2: PF1<Output<PushPull>>,
        motor: motor::Motor,
        usb_dev: UsbDevice<'static, UsbBus<Peripheral<PA11<Alternate<AF14>>, PA12<Alternate<AF14>>>>>,
        msg_tx: Sender<'static, [u8; 32], QUEUE_DEPTH>,
        msg_rx: Receiver<'static, [u8; 32], QUEUE_DEPTH>,
        telem_tx: Sender<'static, [u8; 32], TELEM_DEPTH>,
        telem_rx: Receiver<'static, [u8; 32], TELEM_DEPTH>,
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
    A5 - ESTOP      (ADC2.AIN14)
    A6 - STEP       (TIM16.1, AF1)
    A7 - PHASE_B    (TIM3.2, AF2)
    A8 - DISP_CLK   (I2C3)
    A9 - DIR
    A10 - ON_SW     (TIM8_BKIN)
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
        //let _speed = gpioa.pa5.into_analog();
        let adc1 = adc::Adc1::new();
        let adc2 = adc::Adc2::new();

        // Motor PWM GPIO
        let low_side_a = gpiob.pb3.into_alternate();
        let low_side_b = gpiob.pb0.into_alternate();
        let high_side_a = gpiob.pb6.into_alternate();
        let high_side_b = gpiob.pb8.into_alternate();
        let e_stop = gpioa.pa10.into_alternate();
        //let (mut pwm_a, mut pwm_b) = dp.TIM8.pwm((high_side_a, high_side_b), 30.kHz(), &mut rcc);
        let (fault, (pwm_a, pwm_b)) = dp.TIM8.pwm_advanced(
            (high_side_a, high_side_b), &mut rcc)
            .frequency(30.kHz())
            .with_deadtime(500.nanos())
            .with_break_pin(e_stop, Polarity::ActiveLow)
            .finalize();
        let mut pwm_a = pwm_a.into_complementary(low_side_a);
        let mut pwm_b = pwm_b.into_complementary(low_side_b);
        // Enable UEV as trigger
        unsafe { (*hal::stm32::TIM8::ptr()).cr2.write(|w| w.mms2().bits(2)) };
        pwm_a.set_duty(0);
        pwm_a.enable();
        pwm_b.set_duty(0);
        pwm_b.enable();

        let mut motor = motor::Motor {
            pwm_fwd: pwm_a,
            pwm_rev: pwm_b,
        };

        // End-stops
        let mut endstop = gpioa.pa5.into_pull_up_input();
        endstop.make_interrupt_source(syscfg);
        endstop.enable_interrupt(exti);

        // LED
        let mut led1 = gpiof.pf0.into_push_pull_output();
        let mut led2 = gpiof.pf1.into_push_pull_output();
        led1.set_low().unwrap();
        led2.set_low().unwrap();

        // GPIO
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

        let mut control = controller::Controller::new(motor_enc, fault, endstop);
        control.torque.set_target(0.000);
        control.speed.set_target(00.0);
        control.torque.enabled = false;

        let (s, r) = make_channel!([u8; 32], QUEUE_DEPTH);
        let (ts, tr) = make_channel!([u8; 32], TELEM_DEPTH);

        // Enable Interrupts
        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM1_BRK_TIM15);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::ADC1_2);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::USB_LP);
        }

        cmd_task::spawn().unwrap();
        telem_task::spawn().unwrap();
        display_task::spawn().unwrap();

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
                usb_dev,
                msg_rx: r,
                msg_tx: s,
                telem_rx: tr,
                telem_tx: ts,
            }
        )
    }

    #[task(priority = 2, local = [msg_rx], shared = [control])]
    async fn cmd_task(mut ctx: cmd_task::Context) {
        info!("starting cmd loop.");
        let mut control = ctx.shared.control;
        let msg_rx = ctx.local.msg_rx;

        loop {
            if let Ok(bytes) = msg_rx.recv().await {
                control.lock(|control| {
                    match Message::from_bytes(&bytes) {
                        Some(Message::SetSpeed(speed)) => {
                            let speed = clamp(speed, -1000.0, 1000.0);
                            control.speed.set_target(speed);
                            info!("Set Speed {}", speed);
                        },
                        Some(Message::SetPosition(pos)) => {
                            let pos = clamp(pos, -10000, 10000);
                            control.position.set_target(pos);
                            info!("Set Position {}", pos);
                        },
                        Some(Message::Enable(en)) => {
                            control.torque.enabled = en;
                            info!("Enable {}", en);
                        },
                        _ => {}
                    }
                });
            }
        }
    }

    #[task(priority = 1, local = [telem_rx], shared = [usb_serial, control])]
    async fn telem_task(mut ctx: telem_task::Context) {
        info!("starting telem loop.");
        let mut control = ctx.shared.control;
        let mut serial = ctx.shared.usb_serial;
        let telem_rx = ctx.local.telem_rx;

        let mut enabled = false;
        loop {
            control.lock(|ctrl| enabled = ctrl.torque.enabled);

            if let Ok(msg) = telem_rx.recv().await {
                if enabled {
                    serial.lock(|serial| {
                        //if serial.rts() {
                            serial.write_all(&msg);
                        //}
                    });
                }
            }
        }
    }

    #[task(priority = 1, local = [led2], shared = [control])]
    async fn display_task(mut ctx: display_task::Context) {
        info!("starting disp loop.");
        //let adc = ctx.local.adc2;
        let mut control = ctx.shared.control;
        let led2 = ctx.local.led2;

        let mut i = 0;

        loop {
            //adc.start_conversion();
            //let voltage = adc::Adc2::sample_to_input_voltage(adc.read_sample());

            //adc.start_conversion();
            //let potentiometer = adc::Adc2::sample_to_volts(adc.read_sample());
            //control.lock(|ctrl| ctrl.position.update_abs_position(potentiometer));

            led2.set_high().unwrap();
            Mono::delay(400.millis()).await;
            led2.set_low().unwrap();
            Mono::delay(100.millis()).await;

            i += 1;
        }
    }


    #[task(binds = TIM1_BRK_TIM15, priority=4, local = [adc2, led1, telem_tx], shared = [control, usb_serial])]
    fn timer_interrupt(mut ctx: timer_interrupt::Context) {
        controller::Speed::clear_timer();

        let adc = ctx.local.adc2;
        adc.start_conversion();

        let led1 = ctx.local.led1;

        //led2.set_low().unwrap();

        let tlm_tx = ctx.local.telem_tx;
        let mut ctrl = ctx.shared.control;

        let potentiometer = adc::Adc2::sample_to_volts(adc.read_sample());
        ctrl.lock(|ctrl| {
            let now = Mono::now();
            let torque = ctrl.speed.control_cycle();
            ctrl.torque.set_target(torque);

            let output = !ctrl.fault.is_fault_active();
            led1.set_state(output.into());

            let mut buffer = [0u8; 32];

            buffer[0] = b'z';
            buffer[1..5].copy_from_slice(&now.ticks().to_be_bytes());
            buffer[5..9].copy_from_slice(&ctrl.torque.current.to_be_bytes());
            buffer[9..13].copy_from_slice(&ctrl.speed.command_torque.to_be_bytes());
            buffer[13..17].copy_from_slice(&ctrl.speed.last_out.p.to_be_bytes());
            buffer[17..21].copy_from_slice(&ctrl.speed.last_out.i.to_be_bytes());
            buffer[21..25].copy_from_slice(&ctrl.speed.last_out.d.to_be_bytes());
            buffer[25..29].copy_from_slice(&ctrl.speed.rpm.to_be_bytes());
            buffer[29] = b'\n';
            tlm_tx.try_send(buffer);

            ctrl.position.cycle += 1;
            if ctrl.position.cycle == 10 {
                ctrl.position.cycle = 0;

                let position = ctrl.speed.last_position;
                let speed = ctrl.position.control_cycle(position as i32);
                ctrl.speed.set_target(speed);

                buffer[0] = b'p';
                buffer[1..5].copy_from_slice(&now.ticks().to_be_bytes());
                buffer[5..9].copy_from_slice(&ctrl.position.last_out.output.to_be_bytes());
                buffer[9..13].copy_from_slice(&ctrl.position.last_out.p.to_be_bytes());
                buffer[13..17].copy_from_slice(&ctrl.position.last_out.i.to_be_bytes());
                //buffer[17..21].copy_from_slice(&ctrl.position.last_out.d.to_be_bytes());
                buffer[17..21].copy_from_slice(&ctrl.position.abs_pos.to_be_bytes());
                buffer[21..25].copy_from_slice(&ctrl.position.pid.setpoint.to_be_bytes());
                buffer[25..29].copy_from_slice(&ctrl.speed.last_position.to_be_bytes());
                buffer[29] = b'\n';
                tlm_tx.try_send(buffer);
            }

            ctrl.position.update_abs_position(potentiometer);
        });
    }

    #[task(binds = ADC1_2, priority=5, shared = [control], local = [motor, adc1])]
    fn adc_interrupt(mut ctx: adc_interrupt::Context) {
        let motor = ctx.local.motor;
        let adc = ctx.local.adc1;
        let sample =adc.read_sample();

        let mut ctrl = ctx.shared.control;
        ctrl.lock(|ctrl| {
            let duty_cycle = ctrl.torque.control_cycle(sample);
            motor.set_duty_cycle(duty_cycle);
        });

    }

    #[task(binds=USB_LP, priority=3, local = [msg_tx, usb_dev], shared = [usb_serial])]
    fn usb_interrupt(mut ctx: usb_interrupt::Context) {
        let mut msg_tx = ctx.local.msg_tx;
        let mut usb_dev = ctx.local.usb_dev;
        let mut serial = ctx.shared.usb_serial;

        serial.lock(|serial| {
            if usb_dev.poll(&mut [serial])  {
                let mut data = [0u8; 32];
                match serial.read(&mut data) {
                    Ok(n) => {
                        _ = msg_tx.try_send(data)
                    }
                    _ => { }
                }
            }
        });
    }


    fn clamp<T: PartialOrd>(v :T, min: T, max: T) -> T {
        match v {
            v if v < min => min,
            v if v > max => max,
            _ => v
        }
    }


}
