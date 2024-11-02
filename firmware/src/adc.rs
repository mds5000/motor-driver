
use cortex_m::asm::nop;
use stm32g4xx_hal::stm32::{ADC12_COMMON, ADC1, ADC2, RCC};

pub struct Adc1 { }

pub struct Adc2 { }

impl Adc1 {
    pub fn new() -> Self {
        let adc = unsafe { &(*ADC1::ptr()) };
        let adc_com = unsafe { &(*ADC12_COMMON::ptr()) };
        let rcc = unsafe { &(*RCC::ptr())};

        rcc.ahb2enr.modify(|_, w| w.adc12en().set_bit());
        //rcc.ahb2rstr.write(|w| w.adc12rst().set_bit());
        //rcc.ahb2rstr.write(|w| w.adc12rst().clear_bit());

        // Enable Common CFG
        adc_com.ccr.write(|w| { w.ckmode().sync_div4() });

        // Exit Deep Power Down & enable Vreg
        adc.cr.modify(|_, w| w.deeppwd().clear_bit().advregen().set_bit());
        // Wait 20us (5000 clk cycles at 128MHz)
        cortex_m::asm::delay(5000);
        //assert!( adc.cr.read().advregen().bit_is_set() );

        // Calibrate ADC
        // 1. Ensure ADEN = 0
        //assert!( adc.cr.read().aden().bit_is_set() );
        // 2. write adcaldif = 0
        //adc.cr.modify(|_, w| w.adcaldif().clear_bit());
        // 3. write ADCAL = 1
        adc.cr.modify(|_, w| w.adcal().calibration());
        // 4. Wait for ADCAL == 0
        while adc.cr.read().adcal().bit_is_set() {}

        // Configure ADC1
        adc.cfgr.write(|w| unsafe {
            w.exten().rising_edge()
             .extsel().bits(0x08) // tim8_trgo2
             .ovrmod().set_bit()
        });
        adc.cfgr2.write(|w| unsafe { w.bits(0) });

        // Set Sample 1 to 47.5cycles = 1.5us
        adc.smpr1.write(|w| w.smp0().cycles47_5());

        // Configure Sequencer, number of conversions
        adc.sqr1.write(|w| unsafe {
            w.l().bits(0)    // One Conversion
             .sq1().bits(0b00001)  // Channel 0
        });

        // Enable EOC Interrupt
        adc.ier.write(|w| w.eocie().set_bit());

        // Enable the ADC
        // 1. Clear the ADRDY bit
        adc.isr.modify(|_, w| w.adrdy().set_bit());
        // 2. Enable the ADC
        adc.cr.modify(|_, w| w.aden().set_bit());
        // 3. Wait for the ADC to become ready
        while adc.isr.read().adrdy().bit_is_clear() {}

        // Start the ADC, waiting for next trigger to occur.
        adc.cr.modify(|_, w| w.adstart().set_bit());

        Adc1 {}
    }

    pub fn read_sample(&self) -> u16 {
        let adc = unsafe { &(*ADC1::ptr()) };

        adc.dr.read().rdata().bits()
    }

}

impl Adc2 {
    pub fn new() -> Self {
        // This must be setup *after* ADC1
        let adc = unsafe { &(*ADC2::ptr()) };

        // Exit Deep Power Down & enable Vreg
        adc.cr.modify(|_, w| w.deeppwd().clear_bit().advregen().set_bit());
        // Wait 20us (5000 clk cycles at 128MHz)
        cortex_m::asm::delay(5000);
        //assert!( adc.cr.read().advregen().bit_is_set() );

        // Calibrate ADC
        // 1. Ensure ADEN = 0
        //assert!( adc.cr.read().aden().bit_is_set() );
        // 2. write adcaldif = 0
        //adc.cr.modify(|_, w| w.adcaldif().clear_bit());
        // 3. write ADCAL = 1
        adc.cr.modify(|_, w| w.adcal().calibration());
        // 4. Wait for ADCAL == 0
        while adc.cr.read().adcal().bit_is_set() {}

        // Configure ADC1
        adc.cfgr.write(|w| w.autdly().set_bit());
        adc.cfgr2.write(|w| unsafe { w.bits(0) });

        // Configure Sequencer, number of conversions
        adc.sqr1.write(|w| unsafe {
            w.l().bits(0)    // One Conversion
             //.sq1().bits(2)  // Channel 2
             .sq2().bits(13)        // Channel 13
        });

        // Enable the ADC
        // 1. Clear the ADRDY bit
        adc.isr.write(|w| w.adrdy().set_bit());
        // 2. Enable the ADC
        adc.cr.modify(|_, w| w.aden().set_bit());
        // 3. Wait for the ADC to become ready
        while adc.isr.read().adrdy().bit_is_clear() {}

        Adc2 {}
    }

    pub fn start_conversion(&self) {
        let adc = unsafe { &(*ADC2::ptr()) };
        adc.cr.modify(|_, w| w.adstart().set_bit());
    }

    pub fn read_sample(&self) -> u16 {
        let adc = unsafe { &(*ADC2::ptr()) };
        while adc.isr.read().eoc().bit_is_clear() {}

        adc.dr.read().rdata().bits()
    }

    pub fn sample_to_input_voltage(sample: u16) -> f32 {
        let volts = (sample as f32) * 3.3 / 4095.0;

        volts * 23.0
    }

    pub fn sample_to_volts(sample: u16) -> f32 {
        (sample as f32) * 3.3 / 4095.0
    }

}