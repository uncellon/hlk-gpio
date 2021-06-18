#include <hlk/gpio/gpio.h>
#include <iostream>
#include <unistd.h>

using namespace Hlk;

int main(int argc, char *argv[]) {
    Gpio gpio;
    
    // Register error handler
    gpio.onError.addEventHandler(
        [] (Gpio::Error error) {
            std::cout << "error occurred: ";
            switch (error) {
            case Gpio::Error::FAILED_TO_OPEN:
                std::cout << "failed to open device\n";
                break;
            
            case Gpio::Error::FAILED_TO_SET_DIRECTION:
                std::cout << "set pin mode failed\n";
                break;

            case Gpio::Error::FAILED_TO_SET_VALUE:
                std::cout << "set pin value failed\n";
                break;

            case Gpio::Error::PIN_IS_NOT_IN_OUTPUT_DIRECTION:
                std::cout << "attempt to set value of non-output pin\n";
                break;

            default:
                std::cout << "unknown error\n";
                break;
            }
            exit(EXIT_FAILURE);
        }
    );

    // Register input changed handler
    gpio.onInputChanged.addEventHandler(
        [] (int pin, Gpio::Value value) {
            switch (value) {
            case Gpio::Value::LOW:
                std::cout << pin << " " << "low\n";
                break;

            case Gpio::Value::HIGH:
                std::cout << pin << " " << "high\n";
                break;
            }
        }
    );

    // Open device
    gpio.open("gpiochip0");

    // Set gpio 4 output mode
    gpio.setDirection(4, Gpio::Direction::OUTPUT);

    // Set gpio 4 output value
    gpio.setValue(4, Gpio::Value::LOW);

    // Set gpio 4 input mode
    gpio.setDirection(21, Gpio::Direction::INPUT);

    // Set gpio 4 pull
    gpio.setPull(21, Gpio::Pull::UP);

    char input;
    std::cout << "press 'y' to on, 'n' to off\n";
    while (true) {
        std::cin >> input;
        if (input == 'y') {
            std::cout << "switch to on\n";
            gpio.setValue(4, Gpio::Value::HIGH);
        } else if (input == 'n') {
            std::cout << "switch to off\n";
            std::cout << "n\n";
            gpio.setValue(4, Gpio::Value::LOW);
        } else {
            std::cout << "wrong input\n";
        }
    }

    return EXIT_SUCCESS;
}