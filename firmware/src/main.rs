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

    use controller::CtrlState;
    use message::Message;

    use stm32g4xx_hal::pwm::{FaultMonitor, Polarity, PwmAdvExt};
    use stm32g4xx_hal::syscfg::SysCfgExt;
    use stm32g4xx_hal as hal;

    use hal::prelude::*;
    use hal::time::{RateExtU32, ExtU32};
    use stm32g4xx_hal::gpio::{Alternate, AlternateOD, ExtiPin, Input, Output, PullUp, PushPull, AF14, AF15, AF2, AF8};
    use stm32g4xx_hal::gpio::gpioa::{PA11, PA12, PA10, PA7, PA8};
    use stm32g4xx_hal::gpio::gpiob::{PB4, PB5};
    use stm32g4xx_hal::gpio::gpiof::{PF0, PF1};
    use stm32g4xx_hal::rcc::{Config, PllConfig, PllMDiv, PllNMul, PllRDiv};
    use stm32g4xx_hal::pwr::PwrExt;
    use stm32g4xx_hal::usb::{Peripheral, UsbBus};

    use usb_device::bus::UsbBusAllocator;
    use usb_device::device::UsbDevice;
    use usb_device::prelude::*;
    use usbd_serial::embedded_io::Write;
    use usbd_serial::{SerialPort, USB_CLASS_CDC};

    use rtic_sync::{channel::*, make_channel};
    use rtic_monotonics::systick::prelude::*;
    use rtic_monotonics::systick_monotonic;

    use defmt::{info, warn};

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
        led1: PF0<Output<PushPull>>,
        led2: PF1<Output<PushPull>>,
        motor: motor::Motor,
        usb_dev: UsbDevice<'static, UsbBus<Peripheral<PA11<Alternate<AF14>>, PA12<Alternate<AF14>>>>>,
        msg_tx: Sender<'static, [u8; 32], QUEUE_DEPTH>,
        msg_rx: Receiver<'static, [u8; 32], QUEUE_DEPTH>,
        telem_tx: Sender<'static, [u8; 32], TELEM_DEPTH>,
        telem_rx: Receiver<'static, [u8; 32], TELEM_DEPTH>,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBusType>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dp = ctx.device;
        dp.RCC.apb1enr1.write(|w| w.pwren().set_bit());

        // Disable USB PowerDelivery Pin Function
        dp.PWR.cr3.write(|w| w.ucpd1_dbdis().set_bit());

        let pwr = dp.PWR.constrain().freeze();
        let rcc = dp.RCC.constrain();
        let mut syscfg = dp.SYSCFG.constrain();

        /* PLL Configured for 128MHz from 16MHz HSI */
        let mut pll_cfg = PllConfig::default();
        pll_cfg.m = PllMDiv::DIV_2;
        pll_cfg.n = PllNMul::MUL_32;
        pll_cfg.r = Some(PllRDiv::DIV_2);
        let mut rcc = rcc.freeze(Config::pll().pll_cfg(pll_cfg), pwr);

        // Enable HSI 48MHz Clk for USB
        rcc.enable_hsi48();

        Mono::start(ctx.core.SYST, rcc.clocks.sys_clk.to_Hz());

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpiof = dp.GPIOF.split(&mut rcc);

        // Setup Analog to Digital Conversion
        // This must precede PWM setup as the ADC is used to trigger PWM Timer
        let _isns = gpioa.pa0.into_analog();
        let _vsns = gpioa.pa1.into_analog();
        let adc1 = adc::Adc1::new();

        // Motor PWM & GPIO Initialization
        let low_side_a = gpiob.pb3.into_alternate();
        let low_side_b = gpiob.pb0.into_alternate();
        let high_side_a = gpiob.pb6.into_alternate();
        let high_side_b = gpiob.pb8.into_alternate();
        let e_stop = gpioa.pa10.into_alternate();
        let (fault, (pwm_a, pwm_b)) = dp.TIM8.pwm_advanced(
            (high_side_a, high_side_b), &mut rcc)
            .frequency(30.kHz())
            // Testing shows that at least 500ns is needed, but adding some margin
            .with_deadtime(1000.nanos())
            .with_break_pin(e_stop, Polarity::ActiveHigh)
            .finalize();
        let mut pwm_a = pwm_a.into_complementary(low_side_a);
        let mut pwm_b = pwm_b.into_complementary(low_side_b);

        // Enable UEV as trigger for the timer
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
        // PA5 is wired to two limit switches in series. A open circuit may indicate either
        // positive or negative travel exceeded.
        let mut endstop = gpioa.pa5.into_pull_up_input();
        endstop.make_interrupt_source(&mut syscfg);
        endstop.enable_interrupt(&mut dp.EXTI);
        endstop.trigger_on_edge(&mut dp.EXTI, stm32g4xx_hal::gpio::SignalEdge::Rising);

        // LEDs
        let mut led1 = gpiof.pf0.into_push_pull_output();
        let mut led2 = gpiof.pf1.into_push_pull_output();
        led1.set_low().unwrap();
        led2.set_low().unwrap();

        // GPIO
        let mut fan_en = gpiob.pb7.into_push_pull_output();
        fan_en.set_low().unwrap();

        // USB Virtual Serial Port
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
                .manufacturer("MDS5000")
                .product("Flightsim Motor Driver")
                .serial_number("1")])
            .unwrap()
            .device_class(USB_CLASS_CDC)
            .build();

        // Position Encoder
        let enc_a: PB4<Alternate<AF2>> = gpiob.pb4.into_alternate();
        let enc_b: PA7<Alternate<AF2>> = gpioa.pa7.into_alternate();
        let motor_enc = qei::Qei::new(dp.TIM3, enc_a, enc_b);

        let endstop_state = endstop.is_high().unwrap();
        let mut control = controller::Controller::new(motor_enc, fault, endstop);
        // If the endstop is triggered on start-up enter shutdown/error state
        if endstop_state {
            control.shutdown();
        }

        let (s, r) = make_channel!([u8; 32], QUEUE_DEPTH);
        let (ts, tr) = make_channel!([u8; 32], TELEM_DEPTH);

        // Enable Interrupts
        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM1_BRK_TIM15);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::ADC1_2);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::USB_LP);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::EXTI9_5);
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

    /// Command Task (~20Hz)
    /// 
    /// The command task takes messages off the received message queue, decodes them,
    /// and takes the corresponding action (usually update the controller state).
    #[task(priority = 2, local = [msg_rx, led2], shared = [control])]
    async fn cmd_task(mut ctx: cmd_task::Context) {
        info!("starting cmd loop.");
        let mut control = ctx.shared.control;
        let led2 = ctx.local.led2;
        let msg_rx = ctx.local.msg_rx;

        loop {
            if let Ok(bytes) = msg_rx.recv().await {
                led2.toggle();
                control.lock(|control| {
                    match Message::from_bytes(&bytes) {
                        Some(Message::SetSpeed(speed)) => {
                            control.speed.set_target(speed);
                            info!("Set Speed {}", speed);
                        },
                        Some(Message::SetPosition(pos)) => {
                            control.set_position(pos);
                            info!("Set Position {}", pos);
                        },
                        //Some(Message::Enable(en)) => {
                        //    control.torque.enabled = en;
                        //    info!("Enable {}", en);
                        //},
                        Some(Message::Home) => {
                            control.start_homing();
                            info!("Starting Homing...");
                        },
                        _ => {}
                    }
                });
            }
        }
    }

    /// Telemetry Task
    /// 
    /// The telemety task takes messages from the telemetry queue and formats them
    /// for sending over the USB serial console.
    #[task(priority = 1, local = [telem_rx], shared = [usb_serial, control])]
    async fn telem_task(mut ctx: telem_task::Context) {
        info!("starting telem loop.");
        let mut control = ctx.shared.control;
        let mut serial = ctx.shared.usb_serial;
        let telem_rx = ctx.local.telem_rx;

        let mut enabled = false;
        loop {
            control.lock(|ctrl| enabled = ctrl.is_enabled());

            if let Ok(msg) = telem_rx.recv().await {
                if enabled {
                    serial.lock(|serial| {
                        serial.write_all(&msg);
                    });
                }
            }
        }
    }

    /// Display Task
    ///
    /// Encodes the state of the controller into a blinking pattern of two LEDs.
    #[task(priority = 1, local = [led1], shared = [control])]
    async fn display_task(mut ctx: display_task::Context) {
        info!("starting disp loop.");
        let mut control = ctx.shared.control;
        let led1 = ctx.local.led1;

        loop {
            let state = control.lock(|ctrl| ctrl.get_state());
            match state {
                // Double Blink LED1
                CtrlState::Reset => {
                    led1.set_high().unwrap();
                    Mono::delay(100.millis()).await;
                    led1.set_low().unwrap();
                    Mono::delay(100.millis()).await;
                    led1.set_high().unwrap();
                    Mono::delay(100.millis()).await;
                    led1.set_low().unwrap();
                    Mono::delay(700.millis()).await;

                }
                // Solid LED 1
                CtrlState::Homing => {
                    led1.set_low().unwrap();
                    Mono::delay(100.millis()).await;
                }
                // Slow blink LED1
                CtrlState::Homed(_) => {
                    led1.set_high().unwrap();
                    Mono::delay(900.millis()).await;
                    led1.set_low().unwrap();
                    Mono::delay(100.millis()).await;
                }
                // Fast blink LED1
                CtrlState::Error => {
                    led1.set_high().unwrap();
                    Mono::delay(100.millis()).await;
                    led1.set_low().unwrap();
                    Mono::delay(100.millis()).await;
                }
            }

        }
    }


    /// Speed and Position Control Task (1Khz)
    /// 
    /// The speed control loop updates motor torque command once every millisecond.
    /// Every 10 cycles (100Hz) the position control loop updates the requested speed command.
    #[task(binds = TIM1_BRK_TIM15, priority=4, local = [telem_tx, cycle: u32 = 0], shared = [control, usb_serial])]
    fn timer_interrupt(mut ctx: timer_interrupt::Context) {
        controller::Speed::clear_timer();
        let tlm_tx = ctx.local.telem_tx;
        let cycle = ctx.local.cycle;
        let mut ctrl = ctx.shared.control;

        ctrl.lock(|ctrl| {
            // Check E-STOP Condition
            ctrl.check_fault();

            // Update torque setpoint
            let torque = ctrl.speed.control_cycle();
            ctrl.torque.set_target(torque);


            let mut buffer = [0u8; 32];
            ctrl.generate_speed_telem(&mut buffer);
            let _ = tlm_tx.try_send(buffer);

            // Update speed setpoint (every 10 cycles)
            *cycle += 1;
            if *cycle == 10 {
                *cycle = 0;

                let position = ctrl.speed.last_position;
                let target_speed = ctrl.position.control_cycle(position);
                ctrl.speed.set_target(target_speed);

                ctrl.generate_position_telem(&mut buffer);
                let _ = tlm_tx.try_send(buffer);
            }
        });
    }

    /// ADC Conversion Interrupt (30kHz)
    /// 
    /// At each sample of the motor current, update the torque control loop.
    #[task(binds = ADC1_2, priority=5, shared = [control], local = [motor, adc1])]
    fn adc_interrupt(ctx: adc_interrupt::Context) {
        let adc = ctx.local.adc1;
        let sample = adc.read_sample();

        let mut ctrl = ctx.shared.control;
        let duty_cycle = ctrl.lock(|ctrl| ctrl.torque.control_cycle(sample));

        let motor = ctx.local.motor;
        motor.set_duty_cycle(duty_cycle);
    }

    /// External GPIO Interrupt (Endstop)
    ///
    /// If currently homing, this triggers the zero-position
    /// Otherwise, it triggers an immediate shutdown and disables motors.
    #[task(binds = EXTI9_5, priority=6, shared = [control])]
    fn exti_interrupt(ctx: exti_interrupt::Context) {
        let mut ctrl = ctx.shared.control;

        ctrl.lock(|ctrl| {
            ctrl.endstop.clear_interrupt_pending_bit();

            if ctrl.get_state() == CtrlState::Homing {
                let home = ctrl.set_home_position();
                info!("Set Home position: {}", home);
                ctrl.position.set_max_speed(100.0);
                ctrl.set_position(0.0);
                ctrl.torque.set_enabled(true);
            } else {
                ctrl.shutdown();

                let pos = ctrl.speed.get_position();
                warn!("Endstop Triggered: Pos {}", pos);
            }
        });
    }

    /// USB Interrupt
    #[task(binds=USB_LP, priority=3, local = [msg_tx, usb_dev], shared = [usb_serial])]
    fn usb_interrupt(mut ctx: usb_interrupt::Context) {
        let msg_tx = ctx.local.msg_tx;
        let usb_dev = ctx.local.usb_dev;
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
}
