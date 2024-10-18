use pid::Pid;
use stm32g4xx_hal::stm32::{RCC, TIM2, TIM15};
use embedded_hal::PwmPin;

use crate::adc;

pub struct Controller {
    pub torque: Torque,
    pub speed: Speed,
}

impl Controller {
    pub fn new() -> Self {
        Controller {
            torque: Torque::new(),
            speed: Speed::init(),
        }
    }
}

pub struct Torque {
    pub pid: Pid<f32>,
    pub current: f32,
    pub duty_cycle: f32,
    pub enabled: bool,
}

// Torque Controller Settings
// 
// Control Rate: 30kHz
// Output: 0 to 0.999 (duty cycle)
// Input: Current (in amps)
impl Torque {
    pub fn new () -> Self {
        let mut pid = Pid::new(0.0, 0.9999);
        pid.p(1.00, 1.000);
        pid.i(0.10, 0.2);
        //pid.d(0.5, 0.4);

        Torque {
            pid,
            current: 0.0,
            duty_cycle: 0.0,
            enabled: false,
        }
    }

    pub fn set_enabled(&mut self, enable: bool) {
        self.enabled = enable
    }

    pub fn set_target(&mut self, target: f32) {
        self.pid.setpoint(target);
    }

    pub fn next_output(&mut self, measurement: f32) -> f32 {
        let control = self.pid.next_control_output(measurement);

        control.output.max(0.0) 
    }

    pub fn control_cycle(&mut self, sample: u16) -> f32 {
        self.current = 0.95 * self.current + 0.05 * Torque::sample_to_motor_current(sample);
        self.duty_cycle = self.next_output(self.current);

        if self.enabled {
            self.duty_cycle
        } else {
            0.0
        }
    }

    // Input 0 - 4095, 0A at 2047
    // 3.3V / 4095counts
    // 0.015 V/A * 20 V/V = 0.3 V/A
    pub fn sample_to_motor_current(sample: u16) -> f32 {
        let centered_sample = (sample as i32) - 2047;
        let volts = (centered_sample as f32) * 3.3 / 4095.0;

        volts / 0.3
    }
}


pub struct Speed {
    pub rpm: f32,
    pub command_torque: f32,
    pub pid: Pid<f32>,
}

impl Speed {
    pub fn init() -> Self {
        Speed::init_timer();

        let mut pid = Pid::new(0.0, 3.0);
        pid.p(0.1, 3.0);
        pid.i(0.01, 0.5);

        Speed {
            rpm: 0.0,
            command_torque: 0.0,
            pid
        }
    }

    pub fn control_cycle(&mut self) -> f32 {
        self.rpm = self.get_rpm();
        let control = self.pid.next_control_output(self.rpm);

        self.command_torque = control.output.max(0.0);

        self.command_torque
    }

    pub fn clear_timer() {
        let tim = unsafe { &(*TIM15::ptr())};
        tim.sr.write(|w| w.uif().clear_bit());
    }

    pub fn set_target(&mut self, target: f32) {
        self.pid.setpoint(target);
    }

    fn init_timer() {
        let rcc = unsafe { &(*RCC::ptr())};
        let tim = unsafe { &(*TIM15::ptr())};

        // SETUP TIMER 15
        rcc.apb2enr.modify(|_, w| w.tim15en().set_bit());


        tim.psc.write(|w| w.psc().variant(128)); // Div by 128 -> 1MHz
        tim.arr.write(|w| w.arr().variant(1000)); // Count to 1000 -> 1KHz
        tim.dier.write(|w| w.uie().set_bit());
        tim.cr1.write(|w| w.arpe().set_bit()
                                   .cen().set_bit());


        // SETUP TIMER 2
        rcc.apb1enr1.modify(|_, w| w.tim2en().set_bit());

        let tim = unsafe { &(*TIM2::ptr())};
        tim.smcr.write(|w| w.ts().variant(2)); // tim_itr2 == TIM3.trgo
        tim.ccmr1_input().write(|w| w.cc1s().variant(3)); // trc is capture.
        tim.smcr.modify(|_, w| w.sms().variant(0b000).sms_3().set_bit()); // Reset on trigger
        tim.ccer.write(|w| w.cc1e().set_bit()); // Enable capture on CC1
        tim.cr1.write(|w| w.cen().set_bit()); // Enable timer.

    }

    pub fn get_rpm(&self) -> f32 {
        let tim = unsafe { &(*TIM2::ptr())};

        let mut speed = tim.ccr1().read().ccr().bits();

        // Avoid div by zero if stopped.
        if speed == 0 {
            return 0.0;
        }

        128.0 * 60.0e6 / (3200.0 * (speed as f32))
    }
}

