/******************************************************************************
 * 
 * Copyright (C) 2021 Dmitry Plastinin
 * Contact: uncellon@yandex.ru, uncellon@gmail.com, uncellon@mail.ru
 * 
 * This file is part of the Hlk GPIO library.
 * 
 * Hlk GPIO is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as pubblished by the
 * Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * Hlk GPIO is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser Public License for more
 * details
 * 
 * You should have received a copy of the GNU Lesset General Public License
 * along with Hlk GPIO. If not, see <https://www.gnu.org/licenses/>.
 * 
 *****************************************************************************/

#ifndef HLK_GPIO_H
#define HLK_GPIO_H

#include <map>
#include <sys/poll.h>
#include <hlk/events/event.h>

namespace Hlk {

class Gpio {
public:    
    enum class Direction;
    enum class Error;
    enum class Value;
    enum class Pull;

    /**************************************************************************
     * Constructors / Destructors
     *************************************************************************/

    Gpio() = default;
    ~Gpio();

    /**************************************************************************
     * Public methods
     *************************************************************************/

    void open(const std::string &dev);
    void close();

    /**************************************************************************
     * Events
     *************************************************************************/

    Event<Error> onError;
    Event<int, Value> onInputChanged;

    /**************************************************************************
     * Accessors / Mutators
     *************************************************************************/

    void setDirection(int pin, Direction direction);
    void setValue(int pin, Value value);
    Value value(int pin);
    void setPull(int pin, Pull value);

protected:
    /**************************************************************************
     * Protected methods
     *************************************************************************/

    void polling();
    void writeSafeInterrupt();

    /**************************************************************************
     * Protected members
     *************************************************************************/

    int m_fd = 0;
    int m_pipes[2];
    int m_rwBytes = 0;

    std::mutex m_pfdsMutex;
    std::mutex m_ocMutex;
    std::mutex m_rwMutex;

    std::map<int, int> m_fdsByPins; /// for quick access to GPIO num from polling
    std::map<int, int> m_pinsByFds; /// for quick access to GPIO num from polling
    std::map<int, Direction> m_directionsByPins;
    std::map<int, unsigned int> m_valuesByFds;

    std::thread *m_thread = nullptr;
    std::vector<pollfd> m_pfds;
    bool m_running = false;
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