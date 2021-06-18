#include "gpio.h"

#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <sys/ioctl.h>
#include <linux/gpio.h>

namespace Hlk {

Gpio::Gpio() : m_fd(0), m_pollingThread(nullptr), m_threadRunning(false) { }

Gpio::~Gpio() {
    close();
}

void Gpio::open(const std::string &dev) {
    // Close previously opened device
    close();

    // Open device
    std::string fullDev = "/dev/" + dev;
    int fd = ::open(fullDev.c_str(), O_RDWR);
    if (fd == -1) { // error occurred
        onError(Error::FAILED_TO_OPEN);
        return;
    }
    m_fd = fd;

    m_threadRunning = true;
    m_pollingThread = new std::thread(&Gpio::polling, this);
}

void Gpio::close() {
    // Close thread
    if (m_threadRunning && m_pollingThread) {
        m_threadRunning = false;
        m_pollingThread->join();
        delete m_pollingThread;
        m_pollingThread = nullptr;
    }

    // Close fd
    if (m_fd) {
        ::close(m_fd);
        m_fd = 0;
    }
}

void Gpio::setDirection(int pin, Direction mode) {
    // Check device open
    if (m_fd == 0) {
        onError(Error::DEVICE_NOT_OPENED);
        return;
    }

    // Check previous mode
    if (m_fdsByPins[pin] != 0) {
        for (size_t i = 0; i < m_pollFds.size(); ++i) {
            if (m_fdsByPins[pin] == m_pollFds[i].fd) {
                ::close(m_fdsByPins[pin]);
                m_pollFds.erase(m_pollFds.begin() + i);
            }
        }
    }

    // Create empty request
    struct gpio_v2_line_request request;
    memset(&request, 0, sizeof(request));

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

    int ret = ioctl(m_fd, GPIO_V2_GET_LINE_IOCTL, &request);
    if (ret == -1) {
        onError(Error::FAILED_TO_SET_DIRECTION);
        return;
    }

    if (request.fd == -1 || request.fd == 0) {
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
        m_pollFds.push_back(poll_fd);
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
    }

    // Apply config
    int ret = ioctl(m_fdsByPins[pin], GPIO_V2_LINE_SET_CONFIG_IOCTL, &config);
    if (ret == -1) {
        std::string error = strerror(errno);
        onError(Error::FAILED_TO_SET_VALUE);
        return;
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

    // Apply config
    int ret = ioctl(m_fdsByPins[pin], GPIO_V2_LINE_SET_CONFIG_IOCTL, &config);
    if (ret == -1) {
        std::string error = strerror(errno);
        onError(Error::FAILED_TO_SET_PULL);
        return;
    }
}

void Gpio::polling() {
    while (m_threadRunning) {
        int ret = poll(m_pollFds.data(), m_pollFds.size(), 3000);
        if (ret > 0) { // process fds events
            for (size_t i = 0; i < m_pollFds.size(); ++i) {
                if (m_pollFds[i].revents & POLLIN) {
                    m_pollFds[i].revents = 0;

                    struct gpio_v2_line_event event;
                    memset(&event, 0 ,sizeof(event));
                    read(m_pollFds[i].fd, &event, sizeof(event));

                    // check previous pin value
                    if (m_valuesByFds[m_pollFds[i].fd] == event.id) {
                        continue;
                    }
                    m_valuesByFds[m_pollFds[i].fd] = event.id;

                    switch (event.id) {
                    case GPIO_V2_LINE_EVENT_RISING_EDGE:
                        onInputChanged(m_pinsByFds[m_pollFds[i].fd], Gpio::Value::HIGH);
                        break;

                    case GPIO_V2_LINE_EVENT_FALLING_EDGE:
                        onInputChanged(m_pinsByFds[m_pollFds[i].fd], Gpio::Value::LOW);
                        break;
                    }
                }
            }
        } else if (ret == -1) { // error occurred

        }
    }
}

} // namespace Hlk