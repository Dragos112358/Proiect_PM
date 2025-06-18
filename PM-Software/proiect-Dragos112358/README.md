
# Software for Money Counting Machine with Bill Storage


The code I wrote controls a system on the Raspberry Pi Pico 2 for detecting
banknotes. It uses presence sensors to detect when a banknote is inserted, and a
TCS230 color sensor to read its RGB values. The motor is started or stopped
based on the banknote’s position, and the RGB frequencies are normalized to
identify the note’s color. The code handles concurrent tasks and delays using
libraries like embassy_executor and embassy_time.


## Project Structure
The project is built using the Rust programming language along with the Embassy library for asynchronous programming. Here are my main tasks explained:

## Main task
-The main function is where I initialize all the hardware peripherals and spawn the async tasks that handle logic like LCD display, coin/bill detection, buzzer alerts, and door lock control.

-Before anything runs, I set up shared global states and the hardware configuration:
```
let bits = 0.0f32.to_bits();
GLOBAL_FLOAT_BITS.store(bits, Ordering::SeqCst);
GLOBAL_MACHINE_STATE.store(bits, Ordering::SeqCst);
```
-I used 2 global variables to store the total sum and the current state of the machine.


### Initialized pins in Main
| **Pin**      | **Purpose**                       |
|--------------|-----------------------------------|
| PIN_2        | Motor back (bill return)          |
| PIN_3        | Door lock control                 |
| PIN_4        | Bill acceptor motor               |
| PIN_5        | Status LED (light)                |
| PIN_6-7      | TCS230 s0 and s1                  |
| PIN_8-9      | Multiplexer controls (s2 and s3)  |
| PIN_10       | Color sensor output               |
| PIN_13       | Reset button                      |
| PIN_14-15    | IR presence sensors               |
| PIN_16-17    | I2C bus (SDA & SCL)               |
| PIN_19-22    | Coin sensors (1, 5, 10, 50 bani)  |
| PIN_26       | Door sensor                       |
| PIN_27       | Buzzer (active LOW)               |
| PIN_28       | LED strip for feedback light      |

-I connect an LCD1602 display via I2C (address 0x27) and initialize it using: ```lcd_init(&mut i2c, lcd_addr).await;```

-I then deploy the 5 tasks that are presented in detail below.

## Bancnote_task

This contains the logic behind activating/deactivating the motor. It uses a state machine that handles the different states involved in processing a banknote.
```
#[derive(PartialEq)]
enum StareMotor {
    Idle,
    Start,
    Read,
    Eject,
    Rejected,
}
```

### Idle State

The ```Idle``` state is the default state. In this state, the two presence sensors are inactive, and the motor is turned off. The system can only transition to the ```Start```  state from here.

### Start State

The ```Start``` state is where the motor becomes active. The transition from idle to start occurs if the first sensor detects a banknote. If no banknote is inserted within 4 seconds, the motor stops and the system returns to the ```Idle``` state. If a banknote is inserted, the system automatically transitions to the ```Read```  state.

### Read State

The ```Read``` state involves reading and identifying the color on the banknote. It uses the following function:
```set_filter(&mut s2, &mut s3, "red").await;```
This sets the color filter using the s2 and s3 pins of the TCS230 color sensor:

| Color Filter | S2   | S3   |
|--------------|------|------|
| Red          | Low  | Low  |
| Green        | High | High |
| Blue         | Low  | High |
| Clear        | High | Low  |

Based on the detected color, the Euclidean distance between the reference color and the measured one is calculated. The color values are normalized by dividing them by the "clear" parameter. The closest match determines the identified banknote.
If the distance is greater than 15, the banknote is considered unrecognized, and the system transitions to the ```Rejected``` state. If the banknote is identified, it transitions to the ```Eject``` state.

### Rejected State

In the ```Rejected``` state, the motor reverses direction for one second to push the banknote out. Afterward, the system waits for 3 seconds in the same state to allow the user to retrieve the banknote. The system returns to the Idle state once the banknote is removed.

### Eject State

In the ```Eject``` state, the motor rotates forward for 1.5 seconds to move the banknote into an internal tray. The amount of money is updated accordingly, and the LCD1602 display shows the updated value.

## Task for coins
I used a task called ```monede_task```. This task is pretty straightforward. I use 4 presence sensors, configured on pins 19, 20, 21, and 22. Each sensor is dedicated to a specific coin type (1, 5, 10, and 50 bani).I use a coin separator, which serves to accurately detect the value of the coins.
``` 
async fn monede_task(
    sensor1: Input<'static>,
    sensor2: Input<'static>,
    sensor3: Input<'static>,
    sensor4: Input<'static>,
)
```
On detection:

-Reads global floating-point value from GLOBAL_FLOAT_BITS

-Calls ```update_float_value(...)```, that updates my global variable GLOBAL_FLOAT_BITS. This variable globally keeps my total sum. Everytime the global variable is updated, I display it on my lcd1602.

-Logs the event using info!

-The loop runs every 20ms using ```Timer::after(Duration::from_millis(20)).await;```. This provides a basic debounce/polling interval to reduce false triggers.

## Reset_task

A reset button is connected to pin 13, which is configured as an input pin. When the button is pressed, the available money amount is reset. The function ```update_float_value(0.00)``` is called, which updates the ```GLOBAL_FLOAT_BITS``` variable to 0. Additionally, the door lock connected to pin 3 is activated (a signal is sent through a relay on channel 3, which triggers the lock). My inputs for this task are the reset button and the door lock.
``` 
async fn reset_task(mut buton_reset: Input<'static>, mut door_lock: Output<'static>) 
```


The loop runs every 100ms using
```Timer::after(Duration::from_millis(100)).await;```
This provides a basic polling interval to reduce CPU usage and power consumption, while still allowing timely detection of button presses.


## LCD1602 Display Task (lcd_task)
This task continuously displays the current state of the machine and the total amount of money detected, using an LCD1602 screen connected via I2C.

### Hardware Configuration
LCD1602 I2C connected to the I2C bus (I2C0). Backlight is enabled by default (0x08). RS and EN are manually controlled to send commands and data. At startup, lcd_init initializes the screen in 4-bit mode.

Every ~100 ms:

-Reads the saved amount (GLOBAL_FLOAT_BITS)

-Reads the machine’s state (GLOBAL_MACHINE_STATE)

-Converts the amount into a ```String 16``` with two decimal places (12.50)

-Displays the state on line 1: ```IDLE```, ```START```, ```READ```, ```EJECT```, ```REJECTED``` 

-Displays the amount followed by “ lei” on line 2

-If the new string is shorter than the previous one, the screen is cleared (lcd_command(..., 0x01)) to avoid problems with display (overwriting not full or display flickering).

### LCD commands

```pub async fn lcd_command<I: AsyncI2c>(i2c: &mut I, addr: u8, cmd: u8) ```

-Purpose: Sends a command byte to the LCD.

-Used for: LCD initialization, cursor setting, clearing display.

-Internally uses: lcd_write_byte(...) with rs = false.

### Data sender to LCD
```pub async fn lcd_data<I: AsyncI2c>(i2c: &mut I, addr: u8, data: u8)```

-Purpose: Sends a character/data byte to be shown on the LCD.

-Used for: Writing visible text.

-Internally uses for ```lcd_write_byte(...)``` with rs = true.

### LCD Initialization

```pub async fn lcd_init<I: AsyncI2c>(i2c: &mut I, addr: u8)```

-Purpose: Initializes the LCD1602 in 4-bit mode with I2C.

Steps:

-Wait 50 ms (LCD power-up delay).

-Send ```0x30``` three times to set 8-bit mode (compatibility sequence).

-Send ```0x20``` to switch to 4-bit mode.

-Send function/configuration commands:

-```0x28``` – 2 lines, 5x8 dots, 4-bit.

-```0x0C``` – Display ON, cursor OFF, blink OFF.

-```0x06``` – Cursor increment, no display shift.

-```0x01``` – Clear display.

### LCD write strings (better than bytes)
```pub async fn lcd_write_str<I: AsyncI2c>(i2c: &mut I, addr: u8, s: &str)```

-Purpose: Writes a whole string to the LCD.

-Iterates over each byte (character) and sends it using lcd_data(...).

### Data transmission via I2C 
```
pub async fn lcd_write_byte<I: AsyncI2c>(
    i2c: &mut I,
    addr: u8,
    byte: u8,
    rs: bool,
    backlight: bool,
)
```

-Purpose: Sends a byte to the LCD via I2C in two nibbles (high + low).

#### Steps:

-Extract high and low nibbles from byte.

#### For each nibble:

-Add backlight + RS + EN bits.

-Toggle EN bit HIGH → LOW to latch the nibble.

-Wait briefly between operations.

#### Important flags:

-rs (Register Select): Command or Data mode.

-backlight: Controls whether backlight bit (0x08) is sent.

### LCD cursor 
```pub async fn lcd_set_cursor<I: AsyncI2c>(i2c: &mut I, addr: u8, col: u8, row: u8)```

-Purpose: Positions the cursor at a specific col and row on the LCD, because I have 2 lines on my LCD, each one of them with 16 characters

-Line offsets: Line 0 starts at 0x00, line 1 at 0x40.

-Sends a ```0x80``` | addr command to set DDRAM address.

### Float to String Conversion
The ```float_to_string(val: f32)``` function:

-Converts the float to a string with 2-digit precision

-Adds a leading 0 if the fractional part is < 10 (e.g., 5.05)

-Correctly rounds the fractional part, including edge cases like 0.9999 → 1.00

### Displayed States
| Internal Code | LCD Message         |
|---------------|---------------------|
| 0.0           | State: IDLE    |
| 1.0           | State: START      |
| 2.0           | State: READ       |
| 3.0           | State: EJECT     |
| 4.0           | State: REJECTED      |
| otherwise     | State: UNKNOWN     |

### Update Interval
The task runs every 100 ms using
```Timer::after(Duration::from_millis(100)).await;```.
This reduces CPU usage and ensures a smooth screen refresh without overloading the system.


## Open Door + Buzzer Task
In this task, I control a buzzer that emits warning beeps when the door is open. I use a presence sensor to detect the state of the door and turn the buzzer on or off accordingly. This is very useful in systems that need to alert when a door has been left open. It announces everyone else in the room that the money in the bank are not safe. The sensor outputs HIGH when the door is open. The buzzer is active LOW: it turns on when I set its pin to LOW and off when it's HIGH.

### What I Do in the Task
```
async fn buzzer_usa_deschisa(mut door_sensor: Input<'static>, mut buzzer: Output<'static>) { ... }
```
### How this task works
-At startup, I wait 500ms to allow the sensor to stabilize.

-When I detect that the door has just opened, I wait another second before activating the buzzer. I don’t want to trigger it from a simple vibration or brief movement.

-While the door stays open, I activate the buzzer like this:

-It beeps for 200ms, then stays silent for 1500ms.

-I repeat this cycle until the door is closed.

-When the door is closed, the buzzer is turned off.

-I check the sensor only every 300ms to avoid unnecessary resource usage.
