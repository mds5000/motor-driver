use defmt::warn;
use embedded_hal::Qei;
use embedded_hal::Direction;
use pid::{ControlOutput, Pid};
use stm32g4xx_hal::gpio::gpioa;
use stm32g4xx_hal::gpio::Input;
use stm32g4xx_hal::gpio::PullUp;
use stm32g4xx_hal::{pwm};
use stm32g4xx_hal::{gpio::{gpioa::{PA5, PA7}, gpiob::PB4, Alternate, AF2}, stm32::{RCC, TIM15, TIM2, TIM3, TIM8}};
use micromath::F32Ext;
use biquad::*;

use crate::qei;

type QeiEnc = qei::Qei<TIM3, PB4<Alternate<AF2>>, PA7<Alternate<AF2>>>;

#[derive(PartialEq, Debug)]
pub enum CtrlState {
    Reset,
    Homing,
    Homed(i32),
    Enabled,

}

pub struct Controller {
    pub state: CtrlState,
    pub torque: Torque,
    pub speed: Speed,
    pub position: Position,
    pub fault: pwm::PwmControl<TIM8, pwm::FaultEnabled>,
    pub endstop: gpioa::PA5<Input<PullUp>>,
}

impl Controller {
    pub fn new(enc: QeiEnc, fault: pwm::PwmControl<TIM8, pwm::FaultEnabled>, endstop: gpioa::PA5<Input<PullUp>>) -> Self {
        Controller {
            state: CtrlState::Reset,
            torque: Torque::new(),
            speed: Speed::init(enc),
            position: Position::init(),
            fault,
            endstop,
        }
    }

    pub fn set_speed(&mut self, velocity: f32) {
        let velocity = velocity.clamp(-1000.0, 1000.0);
        self.speed.set_target(velocity);
    }

    pub fn set_position(&mut self, position: i32) {
        if let CtrlState::Homed(home_pos) = self.state {
            let zero_position: i32 = 12500;
            let pos = home_pos +  zero_position + position.clamp(-13000, 13000);
            self.position.set_target(pos);
        } else {
            warn!("Not homed.")
        }
    }

    pub fn start_homing(&mut self) {
        if self.state != CtrlState::Reset {
            warn!("Not in reset.");
            return;
        }

        self.state = CtrlState::Homing;
        self.torque.enabled = true;
        self.position.set_max_speed(20.0);
        self.position.set_target(-100000);
    }

    pub fn set_home_position(&mut self) -> i32 {
        let home_position = self.speed.get_position();
        self.state = CtrlState::Homed(home_position);

        home_position
    }

    pub fn shutdown(&mut self) {
        self.torque.enabled = false;
        self.state = CtrlState::Reset;
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
        pid.p(0.05, 1.000);
        pid.i(0.001, 0.5);

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

    pub fn control_cycle(&mut self, sample: u16) -> f32 {
        self.current = 0.7 * self.current + 0.3 * Torque::sample_to_motor_current(sample);
        self.duty_cycle = self.pid.next_control_output(self.current).output;

        if self.enabled {
            self.duty_cycle
        } else {
            0.0
        }
    }

    // Input 0 - 4095, 0A at 2047
    // 3.3V / 4095counts
    // 0.004 V/A * 20 V/V = 0.08 V/A
    pub fn sample_to_motor_current(sample: u16) -> f32 {
        let centered_sample = (sample as i32) - 2047;
        let volts = (centered_sample as f32) * 3.3 / 4095.0;

        volts / 0.08
    }
}


pub struct Speed {
    pub rpm: f32,
    pub target_rpm: f32,
    pub command_torque: f32,
    pub last_position: i32,
    pub last_out: ControlOutput<f32>,
    pub pid: Pid<f32>,
    pub encoder: QeiEnc,
    filter: DirectForm2Transposed<f32>,
}

impl Speed {
    pub fn init(enc: QeiEnc) -> Self {
        Speed::init_timer();

        let mut pid = Pid::new(0.0, 12.0);
        pid.p(0.020, 12.0);
        pid.i(0.00005, 5.0);
        pid.d(0.1, 5.0);

        let coeffs = Coefficients::<f32>::from_params(Type::LowPass, 1.khz(), 5.hz(), Q_BUTTERWORTH_F32).unwrap();
        let mut filter = DirectForm2Transposed::<f32>::new(coeffs);

        Speed {
            rpm: 0.0,
            target_rpm: 0.0,
            command_torque: 0.0,
            last_position: 0,
            last_out: ControlOutput{p: 0.0, i: 0.0, d: 0.0, output: 0.0},
            pid,
            encoder: enc,
            filter,
        }
    }

    pub fn control_cycle(&mut self) -> f32 {
        let rpm = self.get_rpm();

        if rpm.abs() < 1300.0 {
            self.rpm = self.filter.run(rpm);
        }
        
        // Feed forward term was derived from current vs torque testing done on the motor
        // About 1 Amp is necessary just to overcome the internal friction of the motor.
        let feed_forward = self.target_rpm.abs().powf(0.21).copysign(self.target_rpm);
        let control = self.pid.next_control_output(self.rpm);
        self.command_torque = (feed_forward + control.output);
        self.last_out = control;

        self.command_torque
    }

    pub fn clear_timer() {
        let tim = unsafe { &(*TIM15::ptr())};
        tim.sr.write(|w| w.uif().clear_bit());
    }

    pub fn set_target(&mut self, target: f32) {
        self.target_rpm = target;
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

    pub fn get_rpm(&mut self) -> f32 {
        let tim2 = unsafe { &(*TIM2::ptr())};

        let pos = unsafe { core::mem::transmute::<u16, i16>(self.encoder.count()) as i32 };
        let dir = self.encoder.direction();

        let speed = match dir {
            // Encoder is wired backwards
            Direction::Upcounting => tim2.ccr1().read().ccr().bits() as i32,
            Direction::Downcounting => -1 * tim2.ccr1().read().ccr().bits() as i32,
        };

        // Avoid div by zero if stopped.
        if speed == 0 {
            return 0.0;
        }

        if pos == self.last_position {
            return 0.0;
        }
        self.last_position = pos;

        128.0 * 60.0e6 / (3200.0 * (speed as f32))
    }

    pub fn get_position(&self) -> i32 {
        self.last_position
    }

}


pub struct Position {
    pub cycle: u32,
    pub last_out: ControlOutput<f32>,
    pub pid: Pid<f32>,
}

impl Position {
    pub fn init() -> Self {
        let mut pid = Pid::new(0.0, 1200.0);
        pid.p(0.16, 1200.0);
        pid.i(0.010, 10.0);

        Position {
            cycle: 0,
            last_out: ControlOutput{p: 0.0, i: 0.0, d: 0.0, output: 0.0},
            pid,
        }
    }

    pub fn control_cycle(&mut self, position: i32) -> f32 {
        self.last_out = self.pid.next_control_output(position as f32);

        self.last_out.output
    }

    pub fn set_max_speed(&mut self, speed: f32) {
        self.pid.output_limit = speed;
    }

    pub fn set_target(&mut self, target: i32) {
        self.pid.setpoint(target as f32);
    }
}
