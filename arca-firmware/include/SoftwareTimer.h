//
// Created by Stephen Welch on 2/22/2024.
//

#ifndef SOFTWARETIMER_H
#define SOFTWARETIMER_H
#include <cstdint>


class Timer {

public:
    Timer(float rate);

    void start();
    void stop();
    void reset();
    bool update();
    void print() const;
    uint32_t getPeriod() const;
    uint32_t getTimeElapsed() const;

private:
    bool running;
    uint32_t period;
    uint32_t startTime;
    uint32_t currentTime;
    uint32_t timeElapsed;
};



#endif //SOFTWARETIMER_H
