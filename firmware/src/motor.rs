use stm32g4xx_hal::{pwm::{ActiveHigh, ComplementaryEnabled, Pwm, C1, C2}, stm32::TIM8};
use micromath::F32Ext;
use embedded_hal::PwmPin;

type PwmPins<C> = Pwm<TIM8, C, ComplementaryEnabled, ActiveHigh, ActiveHigh>;

pub struct Motor {
    pub pwm_fwd: PwmPins<C1>,
    pub pwm_rev: PwmPins<C2>,
}

impl Motor {
    pub fn set_duty_cycle(&mut self, duty: f32) {
        if duty >= 0.0 {
            self.pwm_rev.set_duty(0);
            let dc = self.pwm_fwd.get_max_duty() as f32 * duty.abs();
            self.pwm_fwd.set_duty(dc as u16);
        } else {
            self.pwm_fwd.set_duty(0);
            let dc = self.pwm_rev.get_max_duty() as f32 * duty.abs();
            self.pwm_rev.set_duty(dc as u16);
        }
    }
}

