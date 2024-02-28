//
// Created by Stephen Welch on 2/22/2024.
//

#pragma once

#include <cstdint>


class Timer
{
public:
    Timer(float rate);

    void start();
    void stop();
    void reset();
    bool update();
    void print() const;
    uint32_t getPeriod() const;
    uint32_t getTimeElapsed() const;
    double getActualRate() const;

private:
    bool running;
    uint32_t period;
    uint32_t startTime;
    uint32_t currentTime;
    uint32_t timeElapsed;
};
