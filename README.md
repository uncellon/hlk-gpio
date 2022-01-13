# Homebrew Libraries Kit - Gpio

- [Description](#description)
- [Examples](#examples)
    - [Input watching](#input-watching)
    - [Output changing](#output-changing)

## Description

The library implements a simple interface for accessing the GPIO through the userspace ABI v2.

## Examples

### Input watching

```cpp
// Creating and opening GPIO object
Hlk::Gpio gpio;
gpio.open("/dev/gpiochip0");

// Adding input change handler
gpio.onInputChanged.addEventHandler([] (int pin, Gpio::Value value) {
    switch (value) {
    case Hlk::Gpio::kLow:
        std::cout << "Pin " << pin << " changed to low\n";
        break;
    case Hlk::Gpio::kHigh:
        std::cout << "Pin " << pin << " changed to high\n";
        break;
    }
});

// Configure pin 4 to input
gpio.setDirection(4, Hlk::Gpio::kInput);

// Configure pin 4 bias mode (pull-up or pull-down)
gpio.setBiasMode(4, Hlk::Gpio::kPullUp);
```

### Output changing

```cpp
// Creating and opening GPIO object
Hlk::Gpio gpio;
gpio.open("/dev/gpiochip0");

// Configure pin 5 to output
gpio.setDirection(5, Hlk::Gpio::kOutput);

// Setting pin 5 value to high
gpio.setValue(5, Hlk::Gpio::kHigh);
```