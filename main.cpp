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

#define GPIO_FSEL0_BASE 0x00200000
#define GPIO_FSEL1_BASE 0x00200004
#define GPIO_SET0_BASE 0x0020001C
#define GPIO_CLR0_BASE 0x00200028
#define GPIO_LEVEL0_BASE 0x00200034

#define ACCESS(base, offset) *(volatile uint32_t*)((uint32_t)base + offset)
#define GPIO_INIT(bitMask, setAddr, clrAddr, readAddr, fnselAddr, fnselBit) {bitMask, setAddr, clrAddr, readAddr, fnselAddr, fnselBit}

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
};

class GPIOController
{
    public:
        virtual ~GPIOController();
        static GPIOController* getInstance();
        void setMode(uint32_t gpioBitMask, bool out);
        void set(uint32_t gpioBitMask, bool value);
        bool get(uint32_t gpioBitMask);
    private:
        GPIOController();

        static void* peripherals;
        static GPIO* gpio;
};

void* GPIOController::peripherals = NULL;
GPIO* GPIOController::gpio = NULL;

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

    peripherals = mmap(NULL, 0x002FFFFF, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, isBcm2835 ? 0x20000000 : 0x3F000000);
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

void GPIOController::setMode(uint32_t gpioBitMask, bool out)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO* selected = &gpio[i];
        if (selected->bitMask & gpioBitMask) {
            uint32_t fnselMask = 0x00000000;
            for (uint8_t i = 0; i < 29 - selected->fnselBit; i++) {
                fnselMask = (fnselMask << 1) | 0x01;
            }
            fnselMask = (fnselMask << 3);
            for (uint8_t i = 0; i < selected->fnselBit; i++) {
                fnselMask = (fnselMask << 1) | 0x01;
            }
            ACCESS(peripherals, selected->fnselAddr) = (ACCESS(peripherals, selected->fnselAddr) & fnselMask) | ((out ? 0x01 : 0x00) << selected->fnselBit);
            break;
        }
    }
}

void GPIOController::set(uint32_t gpioBitMask, bool value)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO* selected = &gpio[i];
        if (selected->bitMask & gpioBitMask) {
            ACCESS(peripherals, (value ? selected->setAddr : selected->clrAddr)) = selected->bitMask;
            break;
        }
    }
}

bool GPIOController::get(uint32_t gpioBitMask)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO* selected = &gpio[i];
        if (selected->bitMask & gpioBitMask) {
            return ACCESS(peripherals, selected->readAddr) & selected->bitMask;
            break;
        }
    }
	return false;
}

double getLight() {
    uint32_t counter = 0;
    GPIOController* gpioController = GPIOController::getInstance();
    gpioController->setMode(GPIO_18, 1);
    gpioController->set(GPIO_18, 0);
    usleep(SENSOR_RESET_TIME);
    gpioController->setMode(GPIO_18, 0);
    while ((gpioController->get(GPIO_18) == false) && (counter < SENSOR_SCALE)) {
        asm("nop");
        counter++;
    }
    return (SENSOR_SCALE-counter)/(double)SENSOR_SCALE;
}

int main(int argc, char** argv)
{
    try {
        while (true) {
            cout << "LIGHT: " << getLight() << flush;
            usleep(100000);
            cout << "\rLIGHT:         \r";
        }
    } catch (exception &error) {
        cout << "Error: cannot obtain access to peripherals" << endl;
        return 1;
    }

    return 0;
}
