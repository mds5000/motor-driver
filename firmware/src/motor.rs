use stm32g4xx_hal::{pwm::{ActiveHigh, ComplementaryDisabled, ComplementaryEnabled, Pwm, C1, C2}, stm32::TIM8};
use stm32g4xx_hal::gpio::{gpiob, Output, PushPull};
use stm32g4xx_hal::prelude::OutputPin;
use embedded_hal::PwmPin;

type PwmPins<C> = Pwm<TIM8, C, ComplementaryEnabled, ActiveHigh, ActiveHigh>;
//type PwmPins<C> = Pwm<TIM8, C, ComplementaryDisabled, ActiveHigh, ActiveHigh>;

pub struct Motor {
    pub pwm_fwd: PwmPins<C1>,
    pub pwm_rev: PwmPins<C2>,
    pub forward: bool,
}

impl Motor {
    pub fn set_forward(&mut self) {
        self.pwm_fwd.set_duty(0);
        self.pwm_rev.set_duty(0);
        cortex_m::asm::delay(20 * 128); // 20us deadtime
        self.forward = true
    }

    pub fn set_reverse(&mut self) {
        self.pwm_fwd.set_duty(0);
        self.pwm_rev.set_duty(0);
        cortex_m::asm::delay(20 * 128); // 20us deadtime
        self.forward = false
    }

    pub fn set_duty_cycle(&mut self, duty: f32) {
        if self.forward {
            let dc = self.pwm_fwd.get_max_duty() as f32 * duty;
            self.pwm_fwd.set_duty(dc as u16);
        } else {
            let dc = self.pwm_rev.get_max_duty() as f32 * duty;
            self.pwm_rev.set_duty(dc as u16);
        }
    }
}

