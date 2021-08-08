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

#include "gpio.h"

#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

namespace Hlk {

/******************************************************************************
 * Constructors / Destructors
 *****************************************************************************/

Gpio::~Gpio() {
    close();
}

/******************************************************************************
 * Public methods
 *****************************************************************************/

void Gpio::open(const std::string &device) {
    std::unique_lock lock(m_ocMutex);

    // Open device
    std::string devicePath = "/dev/" + device;
    m_fd = ::open(devicePath.c_str(), O_RDWR);
    if (m_fd == -1) { // error occurred
        m_fd = 0;
        onError(Error::FAILED_TO_OPEN);
        return;
    }

    if (pipe(m_pipes) == -1) {
        ::close(m_fd);
        throw std::runtime_error("pipe(...) failed");
    }

    pollfd pfd;
    pfd.fd = m_fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    m_pfds.push_back(pfd);

    m_running = true;
    m_thread = new std::thread(&Gpio::polling, this);
}

void Gpio::close() {
    std::unique_lock lock(m_ocMutex);

    if (!m_running) {
        return;
    }

    m_running = false;
    m_rwMutex.lock();
    writeSafeInterrupt();
    m_rwMutex.unlock();
    m_thread->join();
    delete m_thread;
    m_thread = nullptr;

    ::close(m_fd);
    m_fd = 0;
}

/******************************************************************************
 * Accessors / Mutators
 *****************************************************************************/

void Gpio::setDirection(int pin, Direction mode) {
    // Check device open
    if (m_fd == 0) {
        onError(Error::DEVICE_NOT_OPENED);
        return;
    }

    // Check previous mode
    if (m_fdsByPins[pin] != 0) {
        for (size_t i = 0; i < m_pfds.size(); ++i) {
            if (m_fdsByPins[pin] != m_pfds[i].fd) continue;
            m_rwMutex.lock();
            writeSafeInterrupt();
            m_pfdsMutex.lock();
            ::close(m_fdsByPins[pin]);
            m_pfds[i].fd = 0;
            m_pfdsMutex.unlock();
            m_rwMutex.unlock();
            break;
        }
    }

    // Create empty request
    gpio_v2_line_request request;
    memset(&request, 0, sizeof(gpio_v2_line_request));

    request.offsets[0] = pin;
    request.num_lines = 1;

    switch (mode) {
    case Direction::INPUT:
        request.config.flags = 
            GPIO_V2_LINE_FLAG_INPUT | 
            GPIO_V2_LINE_FLAG_EDGE_RISING | 
            GPIO_V2_LINE_FLAG_EDGE_FALLING;
        break;

    case Direction::OUTPUT:
        request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
        break;
    }

    if (ioctl(m_fd, GPIO_V2_GET_LINE_IOCTL, &request) == -1) {
        onError(Error::FAILED_TO_SET_DIRECTION);
        return;
    }

    if (request.fd <= 0) {
        onError(Error::FAILED_TO_SET_DIRECTION);
        return;
    }

    // Store info about pin
    m_fdsByPins[pin] = request.fd;
    m_pinsByFds[request.fd] = pin;
    m_directionsByPins[pin] = mode;

    // Add polling if input
    if (mode == Direction::INPUT) {
        pollfd poll_fd;
        poll_fd.fd = request.fd;
        poll_fd.events = POLLIN;

        m_rwMutex.lock();
        writeSafeInterrupt();
        m_pfdsMutex.lock();
        m_pfds.push_back(poll_fd);
        m_pfdsMutex.unlock();
        m_rwMutex.unlock();
    }
}

void Gpio::setValue(int pin, Value value) {
    // Check device open
    if (m_fd == 0) {
        onError(Error::DEVICE_NOT_OPENED);
        return;
    }

    // Check pin mapped
    if (m_fdsByPins[pin] == 0) {
        onError(Error::FAILED_TO_MAP_PIN);
        return;
    }

    // Check pin mode
    if (m_directionsByPins[pin] != Direction::OUTPUT) {
        onError(Error::PIN_IS_NOT_IN_OUTPUT_DIRECTION);
        return;
    }

    // Config struct
    struct gpio_v2_line_config config;
    memset(&config, 0, sizeof(config));

    config.flags = GPIO_V2_LINE_FLAG_OUTPUT;

    switch (value) {
    case Value::LOW:
        config.flags &= ~GPIO_V2_LINE_FLAG_ACTIVE_LOW;
        break;

    case Value::HIGH:
        config.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
        break;
    default:
        onError(Error::INVALID_VALUE);
        return;
    }

    // Apply config
    if (ioctl(m_fdsByPins[pin], GPIO_V2_LINE_SET_CONFIG_IOCTL, &config) == -1) {
        onError(Error::FAILED_TO_SET_VALUE);
        return;
    }
}

Gpio::Value Gpio::value(int pin) {
    // Prepare request
    struct gpio_v2_line_values lineValues;
    memset(&lineValues, 0, sizeof(gpio_v2_line_values));
    lineValues.mask = 1;

    // Request
    if (ioctl(m_fdsByPins[pin], GPIO_V2_LINE_GET_VALUES_IOCTL, &lineValues) == -1) {
        onError(Error::FAILED_TO_GET_VALUE);
        return Value::IDLE;
    }
    
    // Check value
    if (lineValues.bits) {
        return Value::HIGH;
    } else {
        return Value::LOW;
    }
}

void Gpio::setPull(int pin, Pull pull) {
    // Check device open
    if (m_fd == 0) {
        onError(Error::DEVICE_NOT_OPENED);
        return;
    }

    // Check pin mapped
    if (m_fdsByPins[pin] == 0) {
        onError(Error::FAILED_TO_MAP_PIN);
        return;
    }

    // Check pin mode
    if (m_directionsByPins[pin] != Direction::INPUT) {
        onError(Error::PIN_IS_NOT_IN_OUTPUT_DIRECTION);
        return;
    }

    // Config struct
    struct gpio_v2_line_config config;
    memset(&config, 0, sizeof(config));

    config.flags = 
        GPIO_V2_LINE_FLAG_INPUT | 
        GPIO_V2_LINE_FLAG_EDGE_RISING | 
        GPIO_V2_LINE_FLAG_EDGE_FALLING;

    switch (pull) {
    case Pull::UP:
        config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
        break;

    case Pull::DOWN:
        config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
        break;
    }

    // Lock thread to repoll
    m_rwMutex.lock();
    writeSafeInterrupt();
    m_pfdsMutex.lock();

    // Apply config
    if (ioctl(m_fdsByPins[pin], GPIO_V2_LINE_SET_CONFIG_IOCTL, &config) == -1) {
        onError(Error::FAILED_TO_SET_PULL);
        return;
    }

    m_pfdsMutex.unlock();
    m_rwMutex.unlock();
}

/******************************************************************************
 * Protected methods
 *****************************************************************************/

void Gpio::polling() {
    int ret = 0;
    size_t i = 1;
    ssize_t bytesRead = 0;
    char tmp[8];
    Value value;

    while (m_running) {
        m_pfdsMutex.lock();
        ret = poll(m_pfds.data(), m_pfds.size(), -1);
        m_pfdsMutex.unlock();

        if (ret == -1) {
            throw std::runtime_error("poll(...) failed");
        }

        if (m_pfds[0].revents & POLLIN) {
            m_rwMutex.lock();
            do {
                bytesRead = read(m_pfds[0].fd, tmp, 8);
                m_rwBytes -= bytesRead;
            } while (m_rwBytes != 0);
            m_rwMutex.unlock();
        }

        m_pfdsMutex.lock();
        for (i = 1; i < m_pfds.size(); ++i) {
            // Erase closed inputs (descriptors)
            if (m_pfds[i].fd == 0) {
                m_pfds.erase(m_pfds.begin() + i);
                continue;
            }

            // Skip if not POLLIN
            if (!m_pfds[i].revents & POLLIN) continue;

            m_pfds[i].revents = 0;

            struct gpio_v2_line_event event;
            memset(&event, 0 ,sizeof(gpio_v2_line_event));

            // Skip read errors
            if (read(m_pfds[i].fd, &event, sizeof(event)) == -1) continue;

            // Skip same values
            if (m_valuesByFds[m_pfds[i].fd] == event.id) continue;

            m_valuesByFds[m_pfds[i].fd] = event.id;

            // Parse value
            if (event.id == GPIO_V2_LINE_EVENT_RISING_EDGE) {
                value = Value::HIGH;
            } else if (event.id == GPIO_V2_LINE_EVENT_FALLING_EDGE) {
                value = Value::LOW;
            } else {
                value = Value::IDLE;
            }

            m_pfdsMutex.unlock();
            onInputChanged(m_pinsByFds[m_pfds[i].fd], value);
            m_pfdsMutex.lock();
        }
        m_pfdsMutex.unlock();
    }
}

void Gpio::writeSafeInterrupt() {
    if (++m_rwBytes % 8 == 0) {
        write(m_pipes[1], reinterpret_cast<const void *>("00"), 2);
        ++m_rwBytes;
        return;
    }
    write(m_pipes[1], reinterpret_cast<const void *>("0"), 1);
}

} // namespace Hlk