#ifndef HLK_GPIO_H
#define HLK_GPIO_H

#include <map>
#include <string>
#include <thread>
#include <vector>
#include <sys/poll.h>
#include <hlk/events/event.h>

namespace Hlk {

class Gpio {
public:    
    enum class Direction;
    enum class Error;
    enum class Value;
    enum class Pull;

    Gpio();
    ~Gpio();

    void open(const std::string &dev);
    void close();
    void setDirection(int pin, Direction direction);
    void setValue(int pin, Value value);
    Value value(int pin);
    void setPull(int pin, Pull value);

    Event<Error> onError;
    Event<int, Value> onInputChanged;

protected:
    void polling();

    int m_fd;

    std::map<int, int> m_fdsByPins; /// for quick access to GPIO num from polling
    std::map<int, int> m_pinsByFds; /// for quick access to GPIO num from polling
    std::map<int, Direction> m_directionsByPins;
    std::map<int, unsigned int> m_valuesByFds;

    std::thread *m_pollingThread;
    std::vector<pollfd> m_pollFds;
    bool m_threadRunning;
};

enum class Gpio::Direction {
    INPUT = 1,
    OUTPUT
};

enum class Gpio::Error {
    DEVICE_NOT_OPENED = 1,
    FAILED_TO_OPEN,
    FAILED_TO_SET_DIRECTION,
    FAILED_TO_SET_VALUE,
    FAILED_TO_MAP_PIN,
    PIN_IS_NOT_IN_OUTPUT_DIRECTION,
    FAILED_TO_SET_PULL,
    FAILED_TO_GET_VALUE,
    INVALID_VALUE
};

enum class Gpio::Value {
    IDLE = 0,
    LOW,
    HIGH
};

enum class Gpio::Pull {
    DOWN = 0,
    UP
};

} // namespace Hlk

#endif // HLK_GPIO_H