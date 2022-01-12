/******************************************************************************
 * 
 * Copyright (C) 2021 Dmitry Plastinin
 * Contact: uncellon@yandex.ru, uncellon@gmail.com, uncellon@mail.ru
 * 
 * This file is part of the Hlk Gpio library.
 * 
 * Hlk Gpio is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as pubblished by the
 * Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * Hlk Gpio is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser Public License for more
 * details
 * 
 * You should have received a copy of the GNU Lesset General Public License
 * along with Hlk Gpio. If not, see <https://www.gnu.org/licenses/>.
 * 
 *****************************************************************************/

#ifndef HLK_GPIO_H
#define HLK_GPIO_H

#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sys/poll.h>
#include <hlk/events/event.h>

namespace Hlk {

class Gpio {
public:
    enum Error {
        kDeviceNotOpened,
        kFailedToOpen,
        kFailedToSetDirection,
        kFailedToSetValue,
        kPinIsNotOutput,
        kPinIsNotInput,
        kFailedToSetBiasMode,
        kFailedToGetValue,
        kInvalidValue,
        kAlreadyOpened,
        kFailedToCreatePipe,
        kPollingError
    };

    enum Value {
        kIdle,
        kLow,
        kHigh
    };

    enum Direction {
        kInput,
        kOutput
    };
    
    enum BiasMode {
        kPullDown,
        kPullUp
    };

    /**************************************************************************
     * Constructors / Destructors
     *************************************************************************/

    Gpio();
    ~Gpio();

    /**************************************************************************
     * Methods
     *************************************************************************/

    void open(const std::string &dev);
    void close();

    /**************************************************************************
     * Accessors / Mutators
     *************************************************************************/

    Value value(int pin);
    void setValue(int pin, Value value);

    void setDirection(int pin, Direction direction);

    void setBiasMode(int pin, BiasMode mode);

    /**************************************************************************
     * Events
     *************************************************************************/

    Event<Error> onError;
    Event<int, Value> onInputChanged;

protected:
    /**************************************************************************
     * Methods (Protected)
     *************************************************************************/

    void polling();

    /**************************************************************************
     * Members
     *************************************************************************/

    int m_fd;
    int m_pipe[2];

    std::map<int, int> m_fdsByPins; /// for quick access to GPIO num from polling
    std::map<int, int> m_pinsByFds; /// for quick access to GPIO num from polling
    std::map<int, Direction> m_directionsByPins;
    std::map<int, unsigned int> m_valuesByFds;

    std::thread *m_pollingThread;
    std::vector<pollfd> m_pollFds;
    bool m_threadRunning;

    std::mutex m_interruptMutex;
    std::mutex m_workerThreadMutex;
};

// enum class Gpio::Direction ;

// enum class Gpio::Error ;

// enum class Gpio::Value ;

// enum class Gpio::BiasMode ;

} // namespace Hlk

#endif // HLK_GPIO_H