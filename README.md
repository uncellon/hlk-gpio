# Homebrew Libraries Kit - GPIO

- [Description](#description)
- [Examples](#examples)
    - [Input watching](#input-watching)
    - [Output changing](#output-changing)

## Description

The library implements a simple interface for accessing the GPIO through the [userspace ABI v2](https://github.com/torvalds/linux/blob/v5.10/include/uapi/linux/gpio.h).

## Prerequisites

- Linux kernel >= 5.10
- CMake >= 3.16

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

## License

<img align="right" src="https://www.gnu.org/graphics/lgplv3-with-text-154x68.png">

The library is licensed under [GNU Lesser General Public License 3.0](https://www.gnu.org/licenses/lgpl-3.0.txt):

Copyright © 2021 Dmitry Plastinin

Hlk Events is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as pubblished by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

Hlk Events is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser Public License for more details