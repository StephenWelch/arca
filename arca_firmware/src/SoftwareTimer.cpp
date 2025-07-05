//
// Created by Stephen Welch on 2/22/2024.
//

#include "SoftwareTimer.h"
#include "Arduino.h"

Timer::Timer(float rate) {
    this->period = static_cast<uint32_t>(1000000 / rate);
    this->running = false;
    this->startTime = 0;
    this->currentTime = 0;
    this->timeElapsed = 0;
}

void Timer::start() {
    this->running = true;
    this->reset();
}

void Timer::stop()
{
    this->running = false;
}

void Timer::reset() {
    this->startTime = micros();
}

bool Timer::update()
{
    if(!this->running) return false;

    currentTime = micros();
    const uint32_t timeElapsed = currentTime - this->startTime;
    if(timeElapsed >= this->period)
    {
        this->timeElapsed = timeElapsed;
        this->reset();
        return true;
    }
    return false;
}

uint32_t Timer::getTimeElapsed() const
{
    return this->timeElapsed;
}

uint32_t Timer::getPeriod() const
{
    return this->period;
}

double Timer::getActualRate() const
{
    return 1000000.0 / this->getTimeElapsed();
}


void Timer::print() const
{
    Serial.print("timer_");
    Serial.print(this->getPeriod());
    Serial.print(":");
    Serial.print(this->getActualRate());
    Serial.print(" ");
}