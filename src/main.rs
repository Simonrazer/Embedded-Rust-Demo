#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m::prelude::_embedded_hal_blocking_i2c_WriteRead;
use stm32_hal2::{
    self,
    clocks::{Clocks},
    gpio::{Pin, PinMode, Port},
    timer::{Timer, TimerInterrupt},
    pac::{self, TIM5, TIM2, I2C1},
    i2c::I2c
};
use stm32_hal2::{gpio};

use defmt_rtt as _;
use panic_probe as _;

const BLINK_FREQ: f32 = 1.; // seconds

#[rtic::app(device = pac, peripherals = false)]
mod app {
    use core::f32::consts::PI;
    use defmt::println;
    // This line is required to allow imports inside an RTIC module.
    use super::*;

    //Declare state that is shared between differend tasks => enforces locks on access
    #[shared]
    struct Shared {
        led_iter: usize,
        led_pins: [Pin; 4], 
        timer_i2c: Timer<TIM2>,
        timer_blink: Timer<TIM5>
    }

    //Declare state that is used by only one task
    #[local]
    struct Local {           
        i2c: I2c<I2C1>,
        btn: Pin,
    }

    //"Main Function", initializes all Pins, State and Interrupts
    #[init]
    fn init(_cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = pac::Peripherals::take().unwrap();

        let clock_cfg = Clocks::default();
        clock_cfg.setup().unwrap();

        //Declare LEDs
        let led_pin1 = Pin::new(Port::D, 13, PinMode::Output);
        let led_pin2 = Pin::new(Port::D, 12, PinMode::Output);
        let led_pin3 = Pin::new(Port::D, 15, PinMode::Output);
        let led_pin4 = Pin::new(Port::D, 14, PinMode::Output);

        //Declare Button
        let mut btn = Pin::new(Port::A, 0, PinMode::Input);
        let led_pins : [Pin; 4] = [led_pin1, led_pin2, led_pin3, led_pin4];
        btn.enable_interrupt(gpio::Edge::Either);
        

        //Declare timer for I2C/Blinking behaviours
        let mut timer_blink = Timer::new_tim5(dp.TIM5, BLINK_FREQ * 2., Default::default(), &clock_cfg);
        timer_blink.enable_interrupt(TimerInterrupt::Update);
        timer_blink.disable();

        let mut timer_i2c = Timer::new_tim2(dp.TIM2, BLINK_FREQ * 10., Default::default(), &clock_cfg);
        timer_i2c.enable_interrupt(TimerInterrupt::Update);
        timer_i2c.enable();

        //Declare and set I2C pins
        let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
        scl.output_type(gpio::OutputType::OpenDrain);
        let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4)); 
        sda.output_type(gpio::OutputType::OpenDrain);

        //Grab I2C device
        let mut i2c = I2c::new(dp.I2C1, stm32_hal2::i2c::I2cDevice::One, 100_000, &clock_cfg);
        //Enable the attached IMU
        let r1 = i2c.write_bytes(0b1101011, &[0x20,0b10000011]);
        if r1.is_err() {
            panic!("ERROR WRITING I2C");
        }

        let led_iter:usize = 0; // Counter to advance which LED is blinking on a button press
        return (
            Shared { led_iter, led_pins, timer_i2c, timer_blink },
            Local { i2c, btn },
            init::Monotonics()
        );
    }

    //Register the following function as idle function
    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    // Register the following function as a task to be performed on the TIM5 interrupt, and define the state that it uses
    #[task(binds = TIM5, local=[], shared = [led_iter, led_pins, timer_blink], priority = 1)]
    // When the timer's counter expires, toggle the pin connected to the LED.
    fn blink_isr(mut cx: blink_isr::Context) {
        //Clear the interrupt
        cx.shared.timer_blink.lock(|timer_blink| { //Aquire lock on the timer
            timer_blink.clear_interrupt(TimerInterrupt::Update);
        });

        cx.shared.led_iter.lock(|led_iter| { //Aquire lock on the led iterator
            cx.shared.led_pins.lock(|led_pins| {
                //Toggle the LED
                if led_pins[*led_iter].is_low() {
                    led_pins[*led_iter].set_high();
                } else {
                    led_pins[*led_iter].set_low();
                }  
            }); 
        });
    }

    // Helper function to read from I2C bus
    fn read_acc(i2c: &mut I2c<I2C1>, reg: u8) -> f32{
        let mut buf1 = [0];
        let r1 = i2c.write_read(0b1101011, &[reg], &mut buf1);
        let mut buf2 = [0];
        let r2 = i2c.write_read(0b1101011, &[reg+1], &mut buf2);
        if r1.is_err() || r2.is_err() {
            panic!("I2C READ ERROR");
        }
        return (buf1[0] as i16 + ((buf2[0] as i16)<<8)) as f32 * 0.061/1000_f32;
    }

    // Register the following function as a task to be performed on the TIM2 interrupt, and define the state that it uses
    #[task(binds = TIM2, local=[i2c], shared = [led_pins, timer_i2c], priority = 1)]
    // On a timer, turn on an LED which corresponds to the orientation of the I2C connected IMU
    fn i2c_con(mut cx: i2c_con::Context) {
        //Clear the interrupt
        cx.shared.timer_i2c.lock(|timer_i2c| {
            timer_i2c.clear_interrupt(TimerInterrupt::Update);
        });

        let x = read_acc(cx.local.i2c, 0x28);
        let y = read_acc(cx.local.i2c, 0x2A);

        cx.shared.led_pins.lock(|led_pins| {
            led_pins[0].set_low();
            led_pins[1].set_low();
            led_pins[2].set_low();
            led_pins[3].set_low();

            use micromath::F32Ext; //Import atan2 from an external Library, since most of the standard library is not avaivable
                                   //when running with #[no_std]
            let ag = ((x.atan2(y)+PI)/PI)*2_f32-0.001; //Calculate the angle in a range of 0 = 0°, 4 = 360°
            println!("{0}",ag);  //This prints the current angle to the connected PC Terminal, using DEFMT
            let z = ag as usize; //Trunkate angle to get the index of the matching LED
            led_pins[z].set_high(); //Set that LED high
        
        });   
    }

    // Register the following function as a task to be performed on the EXTI0 (Button GPIO) interrupt, and define the state that it uses
    #[task(binds = EXTI0, local=[btn], shared=[led_iter, timer_blink, timer_i2c, led_pins], priority = 2)]
    fn btn_action(mut cx: btn_action::Context){
        gpio::clear_exti_interrupt(0);

        cx.shared.led_iter.lock(|led_iter| {
            //When the Button is pressed, advance the LED iterator, enable the Blinking Timer, and disable I2C timer
            if cx.local.btn.is_high() {
                cx.shared.led_pins.lock(|led_pins| {
                    led_pins[0].set_low();led_pins[1].set_low();led_pins[2].set_low();led_pins[3].set_low();
                });
                if *led_iter == 3 { *led_iter = 0;}
                else {*led_iter = *led_iter + 1;}

                cx.shared.timer_blink.lock(|timer_blink| {
                    timer_blink.enable();
                });
                cx.shared.timer_i2c.lock(|timer_i2c| {
                    timer_i2c.disable();
                });
            }
            //When the Button is not pressed, disable the Blinking Timer, and enable I2C timer
            else{
                
                cx.shared.timer_blink.lock(|timer_blink| {
                    timer_blink.disable();
                });
                cx.shared.timer_i2c.lock(|timer_i2c| {
                    timer_i2c.enable();
                });
            }
            
        });
        
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}