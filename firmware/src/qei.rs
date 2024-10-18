use embedded_hal;
use stm32g4xx_hal;
use stm32g4xx_hal::rcc;
use stm32g4xx_hal::stm32::{RCC, TIM3};

pub struct Qei<TIM, PINA, PINB> {
    tim: TIM,
    pin_a: PINA,
    pin_b: PINB,
}

impl<TIM: Instance, PINA, PINB> Qei<TIM, PINA, PINB> {
    pub fn new(mut tim: TIM, pin_a: PINA, pin_b: PINB) -> Self {
        unsafe {
            let rcc_ptr = &(*RCC::ptr());
            TIM::enable(rcc_ptr);
            TIM::reset(rcc_ptr);
        }

        tim.setup_qei();

        Qei{ tim, pin_a, pin_b}
    }
}


impl<TIM: Instance, PINA, PINB> embedded_hal::Qei for Qei<TIM, PINA, PINB> {
    type Count = u16;
    fn count(&self) -> Self::Count {
        self.tim.read_count()
    }

    fn direction(&self) -> embedded_hal::Direction {
        if self.tim.read_direction() {
            embedded_hal::Direction::Upcounting
        } else {
            embedded_hal::Direction::Downcounting
        }
    }
}

pub trait Instance: rcc::Enable + rcc::Reset {
    fn setup_qei(&mut self);

    fn read_direction(&self) -> bool;

    fn read_count(&self) -> u16;
}


impl Instance for TIM3 {
    fn setup_qei(&mut self) {
        // Map tim_ti1 and tim_ti2 as inputs
        self.ccmr1_input().write(|w| unsafe { w.cc1s().bits(0b01).cc2s().bits(0b01) });
        self.ccer.write(|w| {
            w.cc1e().set_bit().cc1p().clear_bit();
            w.cc2e().set_bit().cc2p().clear_bit()
        });
        self.smcr.write(|w| unsafe { w.sms().bits(0b011).sms_3().clear_bit() }); 
        self.cr2.write(|w| w.mms().variant(0).mms_3().set_bit()); // Enc Clock Output
        self.cr1.write(|w| w.cen().set_bit());
    }

    fn read_direction(&self) -> bool {
        self.cr1.read().dir().bit_is_clear()
    }

    fn read_count(&self) -> u16 {
        self.cnt.read().cnt().bits() 
    }
}
