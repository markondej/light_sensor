/*
    light_sensor - Raspberry Pi simple light sensor demo

    Copyright (c) 2018, Marcin Kondej
    All rights reserved.

    See https://github.com/markondej/light_sensor

    Redistribution and use in source and binary forms, with or without modification, are
    permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors may be
    used to endorse or promote products derived from this software without specific
    prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
    SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
    TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
    BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
    WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>

#define GPIO_COUNT 2
#define GPIO_4 0x00000010
#define GPIO_18 0x00040000

#define BCM2835_PERIPHERALS_BASE 0x20000000
#define BCM2836_PERIPHERALS_BASE 0x3F000000
#define GPIO_FSEL0_BASE 0x00200000
#define GPIO_FSEL1_BASE 0x00200004
#define GPIO_SET0_BASE 0x0020001C
#define GPIO_CLR0_BASE 0x00200028
#define GPIO_LEVEL0_BASE 0x00200034
#define TCNT_BASE 0x00003004

#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_PWM 2
#define GPIO_MODE_UNKNOWN 3

#define ACCESS(base, offset) *(volatile uint32_t*)((uint32_t)base + offset)
#define ACCESS64(base, offset) *(volatile uint64_t*)((uint32_t)base + offset)
#define GPIO_INIT(bitMask, setAddr, clrAddr, readAddr, fnselAddr, fnselBit) {bitMask, setAddr, clrAddr, readAddr, fnselAddr, fnselBit, GPIO_MODE_UNKNOWN, 20.0, 1.0};

#define SENSOR_SCALE 100000
#define SENSOR_RESET_TIME 100000

using std::string;
using std::exception;
using std::cout;
using std::flush;
using std::endl;

struct GPIO {
    uint32_t bitMask, setAddr, clrAddr, readAddr;
    uint32_t fnselAddr, fnselBit;
    uint8_t mode;
    double pwmPeriod, pwmWidth;
};

struct PWM {
    GPIO* gpio;
    double period, width;
    uint64_t startOffset;
    bool high;
};

class GPIOController
{
    public:
        virtual ~GPIOController();
        static GPIOController* getInstance();
        void setMode(uint32_t gpioBitMask, uint8_t mode);
        void setPwm(uint32_t gpioBitMask, double period, double width);
        void set(uint32_t gpioBitMask, bool high);
        bool get(uint32_t gpioBitMask);
    private:
        GPIOController();
        static void* pwmCallback(void* params);

        static void* peripherals;
        static GPIO* gpio;
        static bool pwm;

        pthread_t pwmThread;
};

void* GPIOController::peripherals = NULL;
GPIO* GPIOController::gpio = NULL;
bool GPIOController::pwm = false;

GPIOController::GPIOController()
{
    bool isBcm2835 = true;

    FILE* pipe = popen("uname -m", "r");
    if (pipe) {
        char buffer[64];
        string machine = "";
        while (!feof(pipe)) {
            if (fgets(buffer, 64, pipe)) {
                machine += buffer;
            }
        }
        pclose(pipe);

        machine = machine.substr(0, machine.length() - 1);
        if (machine != "armv6l") {
            isBcm2835 = false;
        }
    }

    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw exception();
    }

    peripherals = mmap(NULL, 0x002FFFFF, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, isBcm2835 ? BCM2835_PERIPHERALS_BASE : BCM2836_PERIPHERALS_BASE);
    close(memFd);
    if (peripherals == MAP_FAILED) {
        throw exception();
    }

    gpio = new GPIO[GPIO_COUNT];
    gpio[0] = GPIO_INIT(GPIO_4, GPIO_SET0_BASE, GPIO_CLR0_BASE, GPIO_LEVEL0_BASE, GPIO_FSEL0_BASE, 12);
    gpio[1] = GPIO_INIT(GPIO_18, GPIO_SET0_BASE, GPIO_CLR0_BASE, GPIO_LEVEL0_BASE, GPIO_FSEL1_BASE, 24);
}

GPIOController::~GPIOController()
{
    munmap(peripherals, 0x002FFFFF);
    delete [] gpio;
}

GPIOController* GPIOController::getInstance()
{
    static GPIOController instance;
    return &instance;
}

void GPIOController::setMode(uint32_t gpioBitMask, uint8_t mode)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO* selected = &gpio[i];
        if (selected->bitMask & gpioBitMask) {
            selected->mode = mode;
            uint32_t fnselMask = 0x00000000;
            for (uint8_t i = 0; i < 29 - selected->fnselBit; i++) {
                fnselMask = (fnselMask << 1) | 0x01;
            }
            fnselMask = (fnselMask << 3);
            for (uint8_t i = 0; i < selected->fnselBit; i++) {
                fnselMask = (fnselMask << 1) | 0x01;
            }
            uint8_t func;
            switch (mode) {
                case GPIO_MODE_IN:
                    func = 0x00;
                    break;
                case GPIO_MODE_OUT:
                    func = 0x01;
                    break;
                case GPIO_MODE_PWM:
                    func = 0x01;
                    break;
                default:
                    throw exception();
            }
            ACCESS(peripherals, selected->fnselAddr) = (ACCESS(peripherals, selected->fnselAddr) & fnselMask) | (func << selected->fnselBit);
            if (mode == GPIO_MODE_PWM) {
                if (!pwm) {
                    pwm = true;
                    if (pthread_create(&pwmThread, NULL, &GPIOController::pwmCallback, NULL)) {
                        throw exception();
                    }
                }
            } else {
                bool stop = true;
                for (uint8_t j = 0; j < GPIO_COUNT; j++) {
                    if ((i != j) && (gpio[j].mode == GPIO_MODE_PWM)) {
                        stop = false;
                    }
                }
                if (stop) {
                    pwm = false;
                    pthread_join(pwmThread, NULL);
                }
            }
            break;
        }
    }
}

void GPIOController::setPwm(uint32_t gpioBitMask, double period, double width)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO* selected = &gpio[i];
        if (selected->bitMask & gpioBitMask) {
            selected->pwmPeriod = period;
            selected->pwmWidth = width;
            break;
        }
    }
}

void GPIOController::set(uint32_t gpioBitMask, bool high)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO* selected = &gpio[i];
        if (selected->bitMask & gpioBitMask) {
            if (selected->mode != GPIO_MODE_PWM) {
                ACCESS(peripherals, (high ? selected->setAddr : selected->clrAddr)) = selected->bitMask;
            }
            break;
        }
    }
}

bool GPIOController::get(uint32_t gpioBitMask)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO* selected = &gpio[i];
        if (selected->bitMask & gpioBitMask) {
            if (selected->mode != GPIO_MODE_PWM) {
                return ACCESS(peripherals, selected->readAddr) & selected->bitMask;
            }
            break;
        }
    }
	return false;
}

void* GPIOController::pwmCallback(void* params)
{
    PWM* pwmData = new PWM[GPIO_COUNT];
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        pwmData[i].gpio = &gpio[i];
        pwmData[i].startOffset = 0;
    }

    while (pwm) {
        uint64_t currentOffset = ACCESS64(peripherals, TCNT_BASE);
        for (uint8_t i = 0; i < GPIO_COUNT; i++) {
            PWM* selected = &pwmData[i];
            if (selected->gpio->mode != GPIO_MODE_PWM) {
                if (selected->startOffset) {
                    selected->startOffset = 0;
                }
                continue;
            }
            if (!selected->startOffset) {
                ACCESS(peripherals, selected->gpio->setAddr) = selected->gpio->bitMask;
                selected->startOffset = currentOffset;
                selected->period = selected->gpio->pwmPeriod;
                selected->width = selected->gpio->pwmWidth;
                selected->high = true;
            } else {
                uint32_t delta = (uint64_t)(currentOffset - selected->startOffset);
                if (delta >= selected->period * 1000) {
                    ACCESS(peripherals, selected->gpio->setAddr) = selected->gpio->bitMask;
                    selected->startOffset = currentOffset - delta + selected->period * 1000;
                    selected->period = selected->gpio->pwmPeriod;
                    selected->width = selected->gpio->pwmWidth;
                    selected->high = true;
                } else if ((delta >= selected->width * 1000) && selected->high) {
                    ACCESS(peripherals, selected->gpio->clrAddr) = selected->gpio->bitMask;
                    selected->high = false;
                }
            }
        }
        asm("nop");
    }

    delete [] pwmData;
    return NULL;
}

double getLight() {
    uint32_t counter = 0;
    GPIOController* gpioController = GPIOController::getInstance();
    gpioController->setMode(GPIO_4, GPIO_MODE_OUT);
    gpioController->set(GPIO_4, 0);
    usleep(SENSOR_RESET_TIME);
    gpioController->setMode(GPIO_4, GPIO_MODE_IN);
    while ((gpioController->get(GPIO_4) == false) && (counter < SENSOR_SCALE)) {
        asm("nop");
        counter++;
    }
    return (SENSOR_SCALE-counter)/(double)SENSOR_SCALE;
}

int main(int argc, char** argv)
{
    GPIOController* gpioController = GPIOController::getInstance();
    gpioController->setPwm(GPIO_18, 20.0, 0.5);
    gpioController->setMode(GPIO_18, GPIO_MODE_PWM);
    try {
        while (true) {
            int light = (int)(getLight() * 100.0d);
            gpioController->setPwm(GPIO_18, 20.0, (light < 70) ? 2.0 : 0.5);
            cout << "LIGHT: " << light << "%" << flush;
            usleep(100000);
            cout << "\rLIGHT:     \r";
        }
    } catch (exception &error) {
        cout << "Error: cannot obtain access to peripherals" << endl;
        return 1;
    }

    return 0;
}
