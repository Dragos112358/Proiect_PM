#![no_std]
#![no_main]

// Allows spawning asynchronous tasks in the Embassy async runtime
use embassy_executor::Spawner;

//for any kind of timers
use embassy_time::{Duration, Timer, Instant}; 

//// defmt_rtt: logging through Real-Time Transfer (RTT)
// panic_probe: minimal panic handler for better debugging
use {defmt_rtt as _, panic_probe as _};
use defmt::*;

// IRQ (interrupt request) management – handling interrupts
use irqs::Irqs;

//Access to GPIO pins for my Raspberry Pi Pico
use embassy_rp::gpio::{Level, Output, Input, Pull};

// Initializes the RP2040 peripherals with Embassy runtime
use embassy_rp::init;

// I2C peripheral access and configuration for RP2040
use embassy_rp::i2c::{Config as I2cConfig, I2c};

//used for my comunication with LCD1602 display
use embedded_hal_async::i2c::I2c as AsyncI2c;

//used for I2C control of peripherals (LCD1602)
use embassy_rp::peripherals::I2C0;

//to be able to have global variables in float format
use core::sync::atomic::{AtomicU32, Ordering};

//heapless is for String type
use heapless::String;

// Floating-point utility traits and functions for math operations
use num_traits::float::FloatCore;

// Core support for 32-bit float operations/constants
use core::f32;

//I calculate euclidian distance between 2 points, so I need sqrt
use libm::sqrt;
// Store the float as bits in an atomic u32
static GLOBAL_FLOAT_BITS: AtomicU32 = AtomicU32::new(0); //global variable for total sum
static GLOBAL_MACHINE_STATE: AtomicU32 = AtomicU32::new(0); //my current state

//enumeration for my lcd, where I have current row, current column and message displayed
#[derive(Clone)]
enum LcdCommand {
    WriteString { 
        row: u8, 
        col: u8, 
        message: heapless::String<16> 
    },
    Clear,
}

mod irqs;

//My states for reading a banknote
#[derive(PartialEq)]
enum StareMotor {
    Asteptare, //IDLE
    Pornire, //START
    Citire, //READ
    Scoatere, //EJECT
    Respins //REJECTED
}

//constants for my TCS230 sensor
const R_WHITE_REF: f32 = 16000.0;
const G_WHITE_REF: f32 = 16000.0;
const B_WHITE_REF: f32 = 16000.0;
const C_WHITE_REF: f32 = 16000.0;
const TOTAL: f32 = (R_WHITE_REF + G_WHITE_REF + B_WHITE_REF) / 3.0;

//converts a float to string, taking into account the fractional part
async fn float_to_string(val: f32) -> String<16> {
    let mut s: String<16> = String::new();

    let mut int_part = val as i32;
    let frac_val = (val - int_part as f32).abs();

    // Correct round, multiply by 100 for safety
    let mut frac_part = (frac_val * 100.0 + 0.5) as u32;

    // if fractional part is 100, add 1 to natural part
    if frac_part == 100 {
        int_part += if val >= 0.0 { 1 } else { -1 };
        frac_part = 0;
    }

    if int_part < 0 {
        let _ = s.push('-');
    }

    // Writing whole part
    {
        let mut n = int_part.abs() as u32;
        let mut buf = [0u8; 10];
        let mut i = 0;

        if n == 0 {
            let _ = s.push('0');
        } else {
            while n > 0 {
                buf[i] = b'0' + (n % 10) as u8; //take every digit
                n /= 10;
                i += 1;
            }
            while i > 0 {
                i -= 1;
                let _ = s.push(buf[i] as char); //push in buffer
            }
        }
    }

    let _ = s.push('.');

    // Writing fractional part with only 2 digits
    if frac_part < 10 {
        let _ = s.push('0');
    }
    {
        let mut n = frac_part;
        let mut buf = [0u8; 10];
        let mut i = 0;

        if n == 0 {
            let _ = s.push('0');
        } else {
            while n > 0 {
                buf[i] = b'0' + (n % 10) as u8;
                n /= 10;
                i += 1;
            }
            while i > 0 {
                i -= 1;
                let _ = s.push(buf[i] as char);
            }
        }
    }

    s
}


// Sends a command byte to the LCD over I2C using an asynchronous I2C interface.
// This is typically used to control the LCD (clear screen, set cursor).
pub async fn lcd_command<I: AsyncI2c>(i2c: &mut I, addr: u8, cmd: u8) {
    lcd_write_byte(i2c, addr, cmd, false, true).await;
}
//writes data to a specified address
pub async fn lcd_data<I: AsyncI2c>(i2c: &mut I, addr: u8, data: u8) {
    lcd_write_byte(i2c, addr, data, true, true).await;
}

//my function to initialize lcd1602
pub async fn lcd_init<I: AsyncI2c>(i2c: &mut I, addr: u8) {
    let backlight = 0x08; // Bit 3 activates backlight
    let en = 0x04;        // Bit 2 - Enable
    let _rs = 0x00;        // Bit 0 - RS (instruction)
    Timer::after(Duration::from_millis(50)).await;

    // Sends 3 times in a row 0x30 as high nibble (for 8-bit mode)
    for _ in 0..3 {
        let byte = 0x30 | backlight;
        i2c.write(addr, &[byte | en]).await.ok(); //writes address with byte
        Timer::after(Duration::from_micros(1)).await;
        i2c.write(addr, &[byte]).await.ok();
        //wait for 5 millis to give the i2c enough time to make the transmission
        Timer::after(Duration::from_millis(5)).await;
    }

    // Sends 0x20 (4-bit mode)
    let byte = 0x20 | backlight;
    i2c.write(addr, &[byte | en]).await.ok();
    Timer::after(Duration::from_micros(1)).await;
    i2c.write(addr, &[byte]).await.ok();
    Timer::after(Duration::from_millis(5)).await;
    lcd_command(i2c, addr, 0x28).await; // Function set: 4-bit, 2 lines, 5x8 dots
    lcd_command(i2c, addr, 0x0C).await; // Display ON, cursor OFF, blink OFF
    lcd_command(i2c, addr, 0x06).await; // Entry mode set: increment, no shift
    lcd_command(i2c, addr, 0x01).await; // Clear display
    Timer::after(Duration::from_millis(2)).await;

    info!("Am initializat ecranul LCD1602");
}

//function that writes a string by taking each bit
pub async fn lcd_write_str<I: AsyncI2c>(i2c: &mut I, addr: u8, s: &str) {
    for c in s.bytes() {
        lcd_data(i2c, addr, c).await;
    }
}


// Sends a byte (command or data) to an HD44780 LCD via I2C using an I/O expander (e.g., PCF8574).
// The byte is split into two 4-bit nibbles as required by the LCD protocol.
// Parameters:
// - i2c: the asynchronous I2C interface
// - addr: the I2C address of the LCD
// - byte: the byte to send (command or character data)
// - rs: if true, byte is treated as data; if false, as a command
// - backlight: controls whether the LCD backlight is on
pub async fn lcd_write_byte<I: AsyncI2c>(
    i2c: &mut I,
    addr: u8,
    byte: u8,
    rs: bool,
    backlight: bool,
) {
    // Set the backlight bit (0x08 turns the backlight on)
    let bl = if backlight { 0x08 } else { 0x00 };

    // Set the RS (Register Select) bit: 1 = data, 0 = command
    let rs_flag = if rs { 0x01 } else { 0x00 };

    // Split the byte into high and low 4-bit nibbles
    let high = byte & 0xF0;             // high nibble (bits 7–4)
    let low = (byte << 4) & 0xF0;       // low nibble (bits 3–0 shifted to bits 7–4)

    // Send each nibble sequentially
    for nibble in [high, low] {
        // Combine nibble with backlight and RS flags
        let data = nibble | bl | rs_flag;

        // Set the Enable bit (EN = 1)
        let with_en  = data | 0x04;

        // Clear the Enable bit (EN = 0)
        let without = data & !0x04;

        // Send data with EN = 1 to latch the nibble into the LCD
        i2c.write(addr, &[with_en]).await.ok();

        // Short delay to satisfy LCD timing requirements (enable pulse width)
        Timer::after(Duration::from_micros(1)).await;

        // Send data with EN = 0 to complete the enable pulse (falling edge triggers LCD read)
        i2c.write(addr, &[without]).await.ok();
    }

    // Delay to allow the LCD to finish processing (especially for slow commands)
    embassy_time::Timer::after(Duration::from_millis(2)).await;
}

//where I place the cursor
pub async fn lcd_set_cursor<I: AsyncI2c>(i2c: &mut I, addr: u8, col: u8, row: u8) {
    let row_offsets = [0x00, 0x40]; //first and second row
    let addr_cmd = 0x80 | (col + row_offsets[row as usize]); //find current address
    lcd_command(i2c, addr, addr_cmd).await;
}
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let bits = 0.0f32.to_bits();
    GLOBAL_FLOAT_BITS.store(bits, Ordering::SeqCst);
    GLOBAL_MACHINE_STATE.store(bits, Ordering::SeqCst);
    //Initializations
    let led0 = Output::new(p.PIN_4, Level::High); //my motor
    let motor_back = Output::new(p.PIN_2,Level::High); //motor backwards
    let led1 = Output::new(p.PIN_5, Level::High); //lightbulb
    let _s0 = Output::new(p.PIN_6, Level::High);
    let _s1 = Output::new(p.PIN_7, Level::Low);
    let s2 = Output::new(p.PIN_8, Level::Low);
    let s3 = Output::new(p.PIN_9, Level::Low);
    let out = Input::new(p.PIN_10, Pull::None);
    let presence_sensor1 = Input::new(p.PIN_15, Pull::None);
    let presence_sensor2 = Input::new(p.PIN_14, Pull::None);
    let buton_reset = Input::new(p.PIN_13,Pull::Up);
    let moneda_sensor1 = Input::new(p.PIN_19, Pull::None); //pins used for 19,20,21 and 22
    let moneda_sensor5 = Input::new(p.PIN_20, Pull::None);
    let moneda_sensor10 = Input::new(p.PIN_21, Pull::None);
    let moneda_sensor50 = Input::new(p.PIN_22, Pull::None);
    let mut leds = Output::new(p.PIN_28, Level::High);
    let mut door_lock = Output::new(p.PIN_3,Level::High); //door opening
    let mut door_sensor = Input::new(p.PIN_26, Pull::None);//door sensor
    let mut buzzer = Output::new(p.PIN_27,Level::High); //buzzer that activates on open door
    let mut config = I2cConfig::default();
    config.frequency = 100_000;
    let sda = p.PIN_16; //sda and scl for I2C communication with lcd 1602
    let scl = p.PIN_17;
    let mut i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, config);
    let lcd_addr = 0x27; // my lcd address

    //initializing lcd
    lcd_init(&mut i2c, lcd_addr).await;

    //banknote task, that uses TCS230 sensor with s0,s1,s2, s3 and out, 2 presence sensors, leds and motor control
    spawner.spawn(bancnote_task(led0,led1, s2, s3, out, presence_sensor1, presence_sensor2,leds,motor_back)).unwrap();

    //coin task, with 4 presence sensor for my 4 kind of coins
    spawner.spawn(monede_task(moneda_sensor1, moneda_sensor5, moneda_sensor10, moneda_sensor50)).unwrap();

    //on reset, the door lock opens
    spawner.spawn(reset_task(buton_reset,door_lock)).unwrap();

    //lcd task 
    spawner.spawn(lcd_task(i2c, lcd_addr)).unwrap();

    //buzzer spawner (it activates when door sensor doesn't detect anything)
    spawner.spawn(buzzer_usa_deschisa(door_sensor, buzzer)).unwrap();
}
#[embassy_executor::task]
async fn buzzer_usa_deschisa(mut door_sensor: Input<'static>, mut buzzer: Output<'static>) {
    let mut last_door_state = door_sensor.is_high();
    
    // Add initial stabilization time
    Timer::after(Duration::from_millis(500)).await;
    
    loop {
        let current_door_state = door_sensor.is_high();
        if current_door_state && !last_door_state {
            //Wait before closing door
            Timer::after(Duration::from_millis(1000)).await;
        }
        last_door_state = current_door_state;
        
        if current_door_state {
            // Sets buzzer to emit sounds for 200 ms
            buzzer.set_low();
            Timer::after(Duration::from_millis(200)).await;
            buzzer.set_high();
            // Wait more before beeps (1.5 seconds)
            Timer::after(Duration::from_millis(1500)).await;
        } else {
            buzzer.set_high();
            // Rarer checks when door is closed, not to overload CPU
            Timer::after(Duration::from_millis(300)).await;
        }
    }
}

//function that updates my current sum available in bank
async fn update_float_value(new_value: f32) {
    let bits = new_value.to_bits();
    GLOBAL_FLOAT_BITS.store(bits, Ordering::SeqCst);
}

//function to update current state in my state machine (for banknote task)
async fn update_state_machine(new_value: f32) {
    let bits = new_value.to_bits();
    GLOBAL_MACHINE_STATE.store(bits, Ordering::SeqCst);
}

//task to display data that updates on lcd1602
#[embassy_executor::task]
async fn lcd_task(mut i2c: I2c<'static, I2C0, embassy_rp::i2c::Async>, lcd_addr: u8) {
    lcd_init(&mut i2c, lcd_addr).await;
    let mut last_len1: usize = 0;
    let mut last_len2: usize = 0;
    let mut last_state_str: String<32> = String::new(); // What I print on the screen (first line)
    let mut last_float_str: String<32> = String::new(); // Second line
    let mut refresh_counter = 0;
    
    loop {
        let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
        let float_value = f32::from_bits(bits);
        let bits2 = GLOBAL_MACHINE_STATE.load(Ordering::SeqCst);
        let curr_state = f32::from_bits(bits2);

        let float_str = float_to_string(float_value).await;

        let state_str = if curr_state == 0.0 {
            "Stare: ASTEPTARE"
        } else if curr_state == 1.0 {
            "Stare: PORNIRE"
        } else if curr_state == 2.0 {
            "Stare: CITIRE"
        } else if curr_state == 3.0 {
            "Stare: SCOATERE"
        } else if curr_state == 4.0 {
            "Stare: RESPINS"
        } else {
            "Stare necunosc."
        };

        let mut float_with_unit = float_str.clone();
        float_with_unit.push_str(" lei");

        let current_len1 = float_str.len();
        let current_len2 = state_str.len();

        // Force refresh every 10 seconds to recover from corruption
        refresh_counter += 1;
        let force_refresh = refresh_counter >= 100;
        
        // Check if content actually changed OR force refresh
        let content_changed = state_str != last_state_str.as_str() || float_with_unit.as_str() != last_float_str.as_str();
        
        if content_changed || force_refresh {
            // Clear LCD only if text becomes shorter OR force refresh
            if current_len1 < last_len1 || current_len2 < last_len2 || force_refresh {
                lcd_command(&mut i2c, lcd_addr, 0x01).await;
                Timer::after(Duration::from_millis(2)).await;
            }

            // Write first line (state)
            lcd_set_cursor(&mut i2c, lcd_addr, 0, 0).await;
            Timer::after(Duration::from_millis(1)).await;
            lcd_write_str(&mut i2c, lcd_addr, state_str).await;

            // Write second line (float)
            lcd_set_cursor(&mut i2c, lcd_addr, 0, 1).await;
            Timer::after(Duration::from_millis(1)).await;
            lcd_write_str(&mut i2c, lcd_addr, float_with_unit.as_str()).await;

            // Update tracking variables for heapless::String
            last_len1 = current_len1;
            last_len2 = current_len2;
            
            last_state_str.clear();
            last_state_str.push_str(state_str).unwrap();
            
            last_float_str.clear();
            last_float_str.push_str(float_with_unit.as_str()).unwrap();
            
            if force_refresh {
                refresh_counter = 0;
            }
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}
//reset button task
#[embassy_executor::task]
async fn reset_task(mut buton_reset: Input<'static>, mut door_lock: Output<'static>) {
    let mut last_reset_state = buton_reset.is_high(); // I suppose the button is not pressed
    loop {
        // Read current button state
        let current_reset_state = buton_reset.is_high();

        // Find transition from high to low
        if last_reset_state && !current_reset_state {
            info!("Butonul de reset a fost apasat!");
            
            // Wait a bit for debounce
            Timer::after(Duration::from_millis(50)).await;
            //Check again
            if !buton_reset.is_high() {
                //If reset is still pressed, update float value and current state
                update_float_value(0.00).await;
                Timer::after(Duration::from_millis(10)).await;
                
                update_state_machine(0.0).await;
                Timer::after(Duration::from_millis(10)).await;
                
                // Activate and deactivate door lock
                door_lock.set_low();
                Timer::after(Duration::from_millis(3000)).await;
                door_lock.set_high();
                
                // Wait until button is not pressed anymore
                while !buton_reset.is_high() {
                    Timer::after(Duration::from_millis(10)).await;
                }
                
                //Another debounce
                Timer::after(Duration::from_millis(50)).await;
            }
        }
        last_reset_state = current_reset_state; 
        // Wait 100 ms before checking again
        Timer::after(Duration::from_millis(100)).await;
    }
}

//task for coins
#[embassy_executor::task]
async fn monede_task(
    sensor1: Input<'static>,
    sensor2: Input<'static>,
    sensor3: Input<'static>,
    sensor4: Input<'static>,
) {
    let mut last1 = sensor1.is_high();
    let mut last2 = sensor2.is_high();
    let mut last3 = sensor3.is_high();
    let mut last4 = sensor4.is_high();
    //I save the current state
    loop {
        let cur1 = sensor1.is_high();
        let cur2 = sensor2.is_high();
        let cur3 = sensor3.is_high();
        let cur4 = sensor4.is_high();
        //for each state, I test if it becomes different
        if !cur1 && last1 {
            //I found coin valued at 1
            let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
            let float_value = f32::from_bits(bits);
            update_float_value(float_value + 0.01).await;
            info!("Moneda detectata de un ban");
            Timer::after(Duration::from_millis(250)).await; // debounce
        }
        if !cur2 && last2 {
            //coin valued at 5, update the total sum
            let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
            let float_value = f32::from_bits(bits);
            update_float_value(float_value + 0.05).await;
            info!("Moneda de 5 bani");
            Timer::after(Duration::from_millis(250)).await;
        }
        if !cur3 && last3 {
            //coin valued at 10, update the total sum
            let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
            let float_value = f32::from_bits(bits);
            update_float_value(float_value + 0.10).await;
            info!("Moneda de 10 bani");
            Timer::after(Duration::from_millis(250)).await;
        }
        if !cur4 && last4 {
            //coin valued at 50, update current sum
            let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
            let float_value = f32::from_bits(bits);
            update_float_value(float_value + 0.50).await;
            info!("Moneda de 50 de bani");
            Timer::after(Duration::from_millis(250)).await;
        }
        //Update states
        last1 = cur1;
        last2 = cur2;
        last3 = cur3;
        last4 = cur4;
        //Quick pooling
        Timer::after(Duration::from_millis(10)).await; 
    }
}


pub struct RGB {
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

impl RGB {
    //calculate euclidian distance between 2 points (colors)
    async fn distance(&self, other: &RGB) -> f32 {
        let dr = (self.r - other.r).powi(2);
        let dg = (self.g - other.g).powi(2);
        let db = (self.b - other.b).powi(2);
        sqrt(dr as f64 + dg as f64 + db as f64) as f32
    }
}

//Banknote identifer
async fn identify_banknote(red: f32, green: f32, blue: f32, _clear: f32) -> &'static str {
    // Normalize the measured values first
    let total = red + green + blue;
    let red1=red/total;
    let green1=green/total;
    let blue1=blue/total;
    info!("VALOARE RGB: R={} G={} B={}", red1,green1,blue1);
    let note_1 = RGB { r: 0.45336914, g: 0.40454102, b: 0.14208984 }; //bagata in stanga, cu capul in sus, pe stanga
    let note_5 = RGB { r: 0.42146355, g: 0.2494376, b: 0.32909885 }; //merge pe partea cu capul in sus, pe centru
    let note_10 = RGB { r: 0.37144586, g: 0.12401694, b: 0.5045372 }; //pe partea diametral opusa cu capul
    let note_50 = RGB { r: 0.4463373, g: 0.16524702, b:0.38841566 }; //bagata pe partea cu  capul, in sus, in dreapta 
    let note_100 = RGB { r: 0.31297162, g: 0.108156666, b: 0.5788717 }; //bagata cu capul in sus
    let note_200 = RGB { r:0.24673784, g:0.569395, b:0.18386714 }; //cu capul inainte in sus
    let note_500 = RGB { r: 0.1, g: 0.1, b: 0.8 };
    let note_alb =RGB {r:104.0, g:102.0,b:89.438};

    let measured = RGB { r: red1, g: green1, b: blue1 };
    //get all the distances
    let dist_1 = measured.distance(&note_1).await;
    let dist_5 = measured.distance(&note_5).await;
    let dist_10 = measured.distance(&note_10).await;
    let dist_50 = measured.distance(&note_50).await;
    let dist_100 = measured.distance(&note_100).await;
    let dist_200 = measured.distance(&note_200).await;
    let dist_500 = measured.distance(&note_500).await;
    let dist_alb = measured.distance(&note_alb).await;
    //what value I actually return
    let distances = [
        (dist_1, "1"),
        (dist_5, "5"),
        (dist_10, "10"),
        (dist_50, "50"),
        (dist_100, "100"),
        (dist_200, "200"),
        (dist_500, "500"),
        (dist_alb,"necunoscut")
    ];
    //get the minimum distance (that is the prediction for my banknote)
    let (min_dist, note) = distances
        .iter()
        .filter(|(d, _)| !d.is_nan())
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
        .unwrap_or(&(f32::INFINITY, "necunoscut"));
    if *min_dist > 0.3 { //if the measure is inaccurate
        return "necunoscut";
    }
    //here I get the float value of my banknote
    //I update the the total sum accordingly
    let valoare: f32 = note.parse().unwrap_or(0.0);
    if (valoare - 1.0).abs() < f32::EPSILON {
         let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
        let float_value = f32::from_bits(bits);
        update_float_value(float_value + 1.00).await;
    }
    if (valoare - 5.0).abs() < f32::EPSILON {
         let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
        let float_value = f32::from_bits(bits);
        update_float_value(float_value + 5.00).await;
    }
    if (valoare - 10.0).abs() < f32::EPSILON {
         let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
        let float_value = f32::from_bits(bits);
        update_float_value(float_value + 10.00).await;
    }
    if (valoare - 50.0).abs() < f32::EPSILON {
         let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
        let float_value = f32::from_bits(bits);
        update_float_value(float_value + 50.00).await;
    }
    if (valoare - 100.0).abs() < f32::EPSILON {
         let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
        let float_value = f32::from_bits(bits);
        update_float_value(float_value + 100.00).await;
    }
    if (valoare - 200.0).abs() < f32::EPSILON {
         let bits = GLOBAL_FLOAT_BITS.load(Ordering::SeqCst);
        let float_value = f32::from_bits(bits);
        update_float_value(float_value + 200.00).await;
    }

    note
    //"necunoscut"
}

//task for banknotes with state machine included
#[embassy_executor::task]
async fn bancnote_task(
    mut led0: Output<'static>,
    mut led1: Output<'static>,
    mut s2: Output<'static>,
    mut s3: Output<'static>,
    out: Input<'static>,
    presence_sensor1: Input<'static>,
    presence_sensor2: Input<'static>,
    mut leds: Output<'static>,
    mut motor_back: Output<'static>
) {
    let mut stare_motor = StareMotor::Asteptare;
    let mut counter = 0;
    let mut scoatere_timer = Instant::now();
    let mut pornire_timer = Instant::now();
    loop {
        counter += 1;
        if counter == 12 {
            counter = 0;
            //read state
            if stare_motor == StareMotor::Citire {

                //set filter for red (low for s2, low for s3)
                set_filter(&mut s2, &mut s3, "red").await;
                Timer::after(Duration::from_millis(400)).await;
                let red = read_average_frequency(&out,400,1).await as f32 * 1.0 / R_WHITE_REF * TOTAL;
                
                //set filter for green (high for s2, high for s3)
                set_filter(&mut s2, &mut s3, "green").await;
                Timer::after(Duration::from_millis(400)).await;
                let green = read_average_frequency(&out,400,1).await as f32 * 1.0 / G_WHITE_REF * TOTAL;
                
                //set filter for blue(low for s2, high for s3)
                set_filter(&mut s2, &mut s3, "blue").await;
                Timer::after(Duration::from_millis(400)).await;
                let blue = read_average_frequency(&out,400,1).await as f32 * 1.0 / B_WHITE_REF * TOTAL;

                //filter for clear(high for s2, low for s3)
                set_filter(&mut s2, &mut s3, "clear").await;
                Timer::after(Duration::from_millis(400)).await;
                let clear = read_average_frequency(&out,400,1).await as f32 * 1.0 / C_WHITE_REF * TOTAL;

                let max_frequency = 100_000.0;
                let r = normalize_to_rgb(red, max_frequency).await;
                let g = normalize_to_rgb(green, max_frequency).await;
                let b = normalize_to_rgb(blue, max_frequency).await;
                let c = normalize_to_rgb(clear,max_frequency).await;
                //printing values
                info!("VALOARE RGB: R={} G={} B={}, C={}", red,green,blue,clear);
                //here I identify the banknote
                let banknote = identify_banknote(red,green,blue,clear).await;
                //if the banknote is unknown, I eject it. I go in Rejected state
                info!("Bancnota identificata este: {}", banknote);
                if banknote == "necunoscut" {
                    info!(" Bancnota nu a fost recunoscuta. Scoatere automata...");
                    stare_motor=StareMotor::Respins;
                    scoatere_timer = Instant::now();
                    update_state_machine(4.0).await;
                }
                else {
                    //I deposit the banknote inside the bank
                    stare_motor = StareMotor::Scoatere;
                    update_state_machine(3.0).await;
                    led0.set_low(); // start engine
                    //led1.set_high(); //stop the lightbulb
                    scoatere_timer = Instant::now();
                }
            }
        }

        match stare_motor {
            StareMotor::Asteptare => {
                if presence_sensor1.is_low() {
                    update_state_machine(1.0).await;
                    stare_motor = StareMotor::Pornire;
                    led0.set_low(); // start engine
                    leds.set_low(); //leds go red (green for IDLE, red for rest)
                    info!("→ Bancnota detectata pe senzorul 1.");
                }
            }
            //REJECTED state
            StareMotor::Respins => {
                if Instant::now() - scoatere_timer < Duration::from_millis(1000) {
                    // Run the motor backwards (eject banknote)
                    led0.set_high(); //stop the motor to spin forwards
                    motor_back.set_low(); //the motor now spins backwards
                } else {
                    // After 1 second, stop the engine and come back to IDLE state
                    leds.set_high(); // LED verde
                    motor_back.set_high(); //Stop the engine at all
                    info!(" Bancnota respinsa. Resetare la asteptare.");
                    //wait 5 seconds to let the user take the banknote
                    Timer::after(Duration::from_millis(5000)).await;
                    stare_motor = StareMotor::Asteptare;
                    update_state_machine(0.0).await
                }
            }
            //START state
            //my first sensor detects a banknote
            StareMotor::Pornire => {
                if presence_sensor2.is_low() {
                    //led1.set_low();
                    led0.set_high(); // stop engine if the banknote reaches the second sensor
                    stare_motor = StareMotor::Citire; //read
                    info!("→ Bancnota la senzorul 2. Oprire pentru citire.");
                    update_state_machine(2.0).await;
                } else if Instant::now() - pornire_timer >= Duration::from_secs(4) {
                    //if my second sensor doesn't detect the banknote within 4 seconds, it will stop 
                    led0.set_high(); // stop engine
                    pornire_timer = Instant::now();
                    stare_motor = StareMotor::Asteptare;
                    leds.set_high(); //My leds are green now, I am in IDLE state
                    update_state_machine(0.0).await;
                    info!("Timeout la introducere. Motor oprit.");
                }
            }
            //EJECT state
            StareMotor::Scoatere => {
                if Instant::now() - scoatere_timer >= Duration::from_millis(1500) {
                    led0.set_high(); // Stop the engine
                    update_state_machine(0.0).await;
                    stare_motor = StareMotor::Asteptare;
                    leds.set_high();
                    info!("→ Scoatere completa. Resetare.");
                }
            }
            _ => {}
        }

        Timer::after(Duration::from_millis(50)).await;
    }
}
//function that sets my filter colors for tcs230
async fn set_filter(s2: &mut Output<'_>, s3: &mut Output<'_>, color: &str) {
    match color {
        //sets s2 and s3 on low for red
        "red" => {
            s2.set_low();
            s3.set_low();
        }
        //sets s2 for low and s3 for high on blue
        "blue" => {
            s2.set_low();
            s3.set_high();
        }
        //sets s2 and s3 on high for green
        "green" => {
            s2.set_high();
            s3.set_high();
        }
        //sets s2 on high and s3 on low for clear
        "clear" => {
            s2.set_high();
            s3.set_low();
        }
        _ => {}
    }
    
    // Wait time for stabilization
    Timer::after(Duration::from_millis(300)).await;
}

async fn read_frequency(out: &Input<'_>, duration_ms: u64) -> u32 {
    let mut count = 0;
    let mut last_state = out.is_high();
    
    // Time for start and end
    let start = Instant::now();
    let end_time = start + Duration::from_millis(duration_ms);
    
    // Variable for debouncing
    let mut last_transition = start;
    let debounce_time = Duration::from_micros(10);
    
    while Instant::now() < end_time {
        let current_state = out.is_high();
        
        // Rising edge with debouncing
        if current_state && !last_state {
            let now = Instant::now();
            if now.saturating_duration_since(last_transition) >= debounce_time {
                count += 1;
                last_transition = now;
            }
        }
        
        last_state = current_state;
        Timer::after(Duration::from_micros(1)).await;
    }
    
    // Calculates the actual frequency in HZ
    let actual_duration = Instant::now().saturating_duration_since(start);
    let actual_duration_ms = actual_duration.as_millis() as u64; //cast to u64
    if actual_duration_ms > 0 {
        (count as u64 * 1000 / actual_duration_ms) as u32
    } else {
        0
    }
}

// Function to read more probes and take the average
async fn read_average_frequency(out: &Input<'_>, duration_ms: u64, samples: u8) -> u32 {
    let mut readings = heapless::Vec::<u32, 16>::new();
    
    // Take multiple samples
    for i in 0..samples {
        let reading = read_frequency(out, duration_ms).await;
        info!("Sample {}: {}", i, reading);
        readings.push(reading).unwrap_or(());
        
        // Small delay between samples for stability
        if i < samples - 1 {
            Timer::after(Duration::from_millis(50)).await;
        }
    }
    
    // Add outlier rejection before taking median
    if !readings.is_empty() {
        readings.sort_unstable();
        let median_idx = readings.len() / 2;
        let median = readings[median_idx];
        
        //Filter out readings that are more than 25% away from median
        let threshold = median / 4; // 25% threshold
        let filtered: heapless::Vec<u32, 16> = readings
            .iter()
            .filter(|&&x| {
                let diff = if x > median { x - median } else { median - x };
                diff <= threshold
            })
            .cloned()
            .collect();
        
        if !filtered.is_empty() {
            // Return average of filtered values
            let sum: u32 = filtered.iter().sum();
            sum / filtered.len() as u32
        } else {
            // Fallback to median if all readings were filtered out
            median
        }
    } else {
        0
    }
}
//function that normalizez my frequences between 0 and 255
async fn normalize_to_rgb(frequency: f32, max_frequency: f32) -> f32 {
    if frequency > max_frequency {
        255.0
    } else {
        (frequency * 255.0 / max_frequency) as f32
    }
}
