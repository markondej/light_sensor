/*
    rpi_cpp_gpio_control - Raspberry Pi GPIO control library for C++

    Copyright (c) 2019, Marcin Kondej
    All rights reserved.

    See https://github.com/markondej/rpi_cpp_gpio_control

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
#include "mailbox.h"
#include <bcm_host.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>

#define PERIPHERALS_BASE 0x7E000000
#define BCM2835_PERIPHERALS_VIRT_BASE 0x20000000
#define GPIO_FSEL_BASE_OFFSET 0x00200000
#define GPIO_SET0_OFFSET 0x0020001C
#define GPIO_CLR0_OFFSET 0x00200028
#define GPIO_LEVEL0_OFFSET 0x00200034
#define GPIO_PUDCTL_OFFSET 0x00200094
#define DMA0_BASE_OFFSET 0x00007000
#define DMA15_BASE_OFFSET 0x00E05000
#define PWMCLK_BASE_OFFSET 0x001010A0
#define PWM_BASE_OFFSET 0x0020C000

#define BCM2837_MEM_FLAG 0x04
#define BCM2835_MEM_FLAG 0x0C
#define PWM_CHANNEL_RANGE 32
#define DMA_BUFFER_SIZE 1024
#define DMA_FREQUENCY 100000
#define PWM_WRITES_PER_CYCLE 4
#define PAGE_SIZE 4096
#define GPIO_COUNT 26

#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_PWM 2
#define GPIO_MODE_UNKNOWN 3

#define GPIO_PULL_UP 0x02
#define GPIO_PULL_DOWN 0x01

#define SENSOR_SCALE 100000
#define SENSOR_RESET_TIME 100000

using std::string;
using std::exception;
using std::cout;
using std::flush;
using std::endl;

struct ClockRegisters {
    uint32_t ctl;
    uint32_t div;
};

struct GPIOPullUpDownRegisters {
    uint32_t ctl;
    uint32_t clock0;
    uint32_t clock1;
};

struct PWMRegisters {
    uint32_t ctl;
    uint32_t status;
    uint32_t dmaConf;
    uint32_t reserved0;
    uint32_t chn1Range;
    uint32_t chn1Data;
    uint32_t fifoIn;
    uint32_t reserved1;
    uint32_t chn2Range;
    uint32_t chn2Data;
};

struct DMAControllBlock {
    uint32_t transferInfo;
    uint32_t srcAddress;
    uint32_t dstAddress;
    uint32_t transferLen;
    uint32_t stride;
    uint32_t nextCbAddress;
    uint32_t reserved0;
    uint32_t reserved1;
};

struct DMARegisters {
    uint32_t ctlStatus;
    uint32_t cbAddress;
    uint32_t transferInfo;
    uint32_t srcAddress;
    uint32_t dstAddress;
    uint32_t transferLen;
    uint32_t stride;
    uint32_t nextCbAddress;
    uint32_t debug;
};

struct GPIO {
    uint8_t number;
    volatile uint32_t *set, *clr, *read, *fnsel;
    uint32_t fnselBit;
    uint8_t mode;
    double pwmPeriod, pwmWidth;
};

struct PWM {
    uint64_t start, end;
    bool enabled;
    GPIO *gpio;
};

class GPIOController
{
    public:
        virtual ~GPIOController();
        static GPIOController &getInstance();
        static void setDmaChannel(uint8_t dmaChannel);
        void setMode(uint8_t gpioNo, uint8_t mode);
        void setPullUD(uint8_t gpioNo, uint32_t type);
        void setPwm(uint8_t gpioNo, double period, double width);
        void set(uint8_t gpioNo, bool high);
        bool get(uint8_t gpioNo);
    private:
        GPIOController();
        GPIOController(const GPIOController &source);
        GPIOController &operator=(const GPIOController &source);
        static uint32_t getMemoryAddress(volatile void *object);
        static uint32_t getPeripheralAddress(volatile void *object);
        static void *getPeripheral(uint32_t offset);
        static bool allocateMemory(uint32_t size);
        static void freeMemory();
        static void *pwmCallback(void *params);

        static void* peripherals;
        static GPIO* gpio;
        static bool pwmEnabled;
        static uint8_t dmaChannel;
        static uint32_t memSize, memAddress, memHandle;
        static void *memAllocated;
        static int mBoxFd;

        pthread_t pwmThread;
};

void* GPIOController::peripherals = NULL;
GPIO* GPIOController::gpio = NULL;
bool GPIOController::pwmEnabled = false;
uint8_t GPIOController::dmaChannel = 0;
uint32_t GPIOController::memSize = 0;
uint32_t GPIOController::memAddress = 0x00000000;
uint32_t GPIOController::memHandle = 0;
void *GPIOController::memAllocated = NULL;
int GPIOController::mBoxFd = 0;

GPIOController::GPIOController()
{
    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw exception();
    }

    peripherals = mmap(NULL, bcm_host_get_peripheral_size(), PROT_READ | PROT_WRITE, MAP_SHARED, memFd, bcm_host_get_peripheral_address());
    close(memFd);
    if (peripherals == MAP_FAILED) {
        throw exception();
    }

    gpio = new GPIO[GPIO_COUNT];
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        gpio[i] = {
            (uint8_t)(i + 2),
            (uint32_t *)getPeripheral(GPIO_SET0_OFFSET), 
            (uint32_t *)getPeripheral(GPIO_CLR0_OFFSET), 
            (uint32_t *)getPeripheral(GPIO_LEVEL0_OFFSET),
            (uint32_t *)getPeripheral(GPIO_FSEL_BASE_OFFSET + ((uint32_t)(i + 2) * 3) / 30 * sizeof(uint32_t)), 
            ((uint32_t)(i + 2) * 3) % 30, 
            GPIO_MODE_UNKNOWN, 
            20.0, 
            1.0
        };
    }
}

GPIOController::~GPIOController()
{
    if (pwmEnabled) {
        pwmEnabled = false;
        pthread_join(pwmThread, NULL);
    }
    munmap(peripherals, bcm_host_get_peripheral_size());
    delete [] gpio;
}

GPIOController &GPIOController::getInstance()
{
    static GPIOController instance;
    return instance;
}

void GPIOController::setDmaChannel(uint8_t channel) {
    if (channel > 15) {
        throw exception();
    }
    dmaChannel = channel;
}

uint32_t GPIOController::getMemoryAddress(volatile void *object)
{
    return (memSize) ? memAddress + ((uint32_t)object - (uint32_t)memAllocated) : 0x00000000;
}

uint32_t GPIOController::getPeripheralAddress(volatile void *object) {
    return PERIPHERALS_BASE + ((uint32_t)object - (uint32_t)peripherals);
}

void *GPIOController::getPeripheral(uint32_t offset) {
    return (void *)((uint32_t)peripherals + offset);
}

bool GPIOController::allocateMemory(uint32_t size)
{
    if (memSize) {
        return false;
    }
    mBoxFd = mbox_open();
    memSize = size;
    if (memSize % PAGE_SIZE) {
        memSize = (memSize / PAGE_SIZE + 1) * PAGE_SIZE;
    }
    memHandle = mem_alloc(mBoxFd, size, PAGE_SIZE, (bcm_host_get_peripheral_address() == BCM2835_PERIPHERALS_VIRT_BASE) ? BCM2835_MEM_FLAG : BCM2837_MEM_FLAG);
    if (!memHandle) {
        mbox_close(mBoxFd);
        memSize = 0;
        return false;
    }
    memAddress = mem_lock(mBoxFd, memHandle);
    memAllocated = mapmem(memAddress & ~0xC0000000, memSize);
    return true;
}

void GPIOController::freeMemory()
{
    unmapmem(memAllocated, memSize);
    mem_unlock(mBoxFd, memHandle);
    mem_free(mBoxFd, memHandle);

    mbox_close(mBoxFd);
    memSize = 0;
}

void GPIOController::setMode(uint8_t gpioNo, uint8_t mode)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO *selected = &gpio[i];
        if (selected->number == gpioNo) {
            selected->mode = mode;
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
            *selected->fnsel = (*selected->fnsel & ((0xFFFFFFF8 << selected->fnselBit) | (0xFFFFFFFF >> (32 - selected->fnselBit)))) | (func << selected->fnselBit);
            if (mode == GPIO_MODE_PWM) {
                if (!pwmEnabled) {
                    pwmEnabled = true;
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
                    pwmEnabled = false;
                    pthread_join(pwmThread, NULL);
                }
            }
            break;
        }
    }
}

void GPIOController::setPullUD(uint8_t gpioNo, uint32_t type) {
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO *selected = &gpio[i];
        if (selected->number == gpioNo) {
            volatile GPIOPullUpDownRegisters *pud = (GPIOPullUpDownRegisters *)getPeripheral(GPIO_PUDCTL_OFFSET);
            pud->ctl = type;
            usleep(10);
            if (selected->number < 32) {
                pud->clock0 = 0x01 << selected->number;
            } else {
                pud->clock1 = 0x01 << (selected->number - 31);
            }
            usleep(10);
            pud->ctl = 0x00;
            if (selected->number < 32) {
                pud->clock0 = 0x00000000;
            } else {
                pud->clock1 = 0x00000000;
            }
            break;
        }
    }
}

void GPIOController::setPwm(uint8_t gpioNo, double period, double width)
{
    if ((period > 0.0f) && (width > 0.0f) && (width < period)) {
        for (uint8_t i = 0; i < GPIO_COUNT; i++) {
            GPIO *selected = &gpio[i];
            if (selected->number == gpioNo) {
                selected->pwmPeriod = period;
                selected->pwmWidth = width;
                break;
            }
        }
    }
}

void GPIOController::set(uint8_t gpioNo, bool high)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO *selected = &gpio[i];
        if (selected->number == gpioNo) {
            if (selected->mode != GPIO_MODE_PWM) {
                if (high) {
                    *selected->set = 0x01 << selected->number;
                } else {
                    *selected->clr = 0x01 << selected->number;
                }
            }
            break;
        }
    }
}

bool GPIOController::get(uint8_t gpioNo)
{
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        GPIO *selected = &gpio[i];
        if (selected->number == gpioNo) {
            if (selected->mode != GPIO_MODE_PWM) {
                return (bool)(*selected->read & (0x01 << selected->number));
            }
            break;
        }
    }
    return false;
}

void *GPIOController::pwmCallback(void *params)
{
    if (!allocateMemory(DMA_BUFFER_SIZE * sizeof(DMAControllBlock) + (GPIO_COUNT + 1) * sizeof(uint32_t))) {
        return NULL;
    }

    PWM *pwmInfo = new PWM[GPIO_COUNT];
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        pwmInfo[i].gpio = &gpio[i];
        pwmInfo[i].enabled = false;
        pwmInfo[i].start = 0;
    }

    volatile ClockRegisters *pwmClk = (ClockRegisters *)getPeripheral(PWMCLK_BASE_OFFSET);
    pwmClk->ctl = (0x5A << 24) | 0x06;
    usleep(1000);
    pwmClk->div = (0x5A << 24) | (uint32_t)((500 << 12) / (PWM_CHANNEL_RANGE * PWM_WRITES_PER_CYCLE * DMA_FREQUENCY / 1000000.0f));
    pwmClk->ctl = (0x5A << 24) | (0x01 << 4) | 0x06;

    volatile PWMRegisters *pwm = (PWMRegisters *)getPeripheral(PWM_BASE_OFFSET);
    pwm->ctl = 0x00;
    usleep(1000);
    pwm->status = 0x01FC;
    pwm->ctl = (0x01 << 6);
    usleep(1000);
    pwm->chn1Range = PWM_CHANNEL_RANGE;
    pwm->dmaConf = (0x01 << 31) | 0x0707;
    pwm->ctl = (0x01 << 5) | (0x01 << 2) | 0x01;

    uint32_t cbIndex = 0, lastComplete = 0;
    volatile DMAControllBlock *dmaCb = (DMAControllBlock *)memAllocated;
    volatile uint32_t *bitMask = (uint32_t *)((uint32_t)dmaCb + DMA_BUFFER_SIZE * sizeof(DMAControllBlock));
    volatile uint32_t *pwmData = (uint32_t *)((uint32_t)bitMask + GPIO_COUNT * sizeof(uint32_t));

    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        bitMask[i] = 0x01 << pwmInfo[i].gpio->number;
    }

    for (uint8_t i = 0; i < 8; i++) {
        dmaCb[cbIndex].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
        dmaCb[cbIndex].srcAddress = getMemoryAddress(pwmData);
        dmaCb[cbIndex].dstAddress = getPeripheralAddress(&pwm->fifoIn);
        cbIndex++;
    }
    lastComplete = cbIndex;

    bool cbAvailable;
    uint64_t offset = 0;
    while (cbIndex < DMA_BUFFER_SIZE) {
        cbAvailable = true;
        for (uint8_t i = 0; i < GPIO_COUNT; i++) {
            if ((pwmInfo[i].gpio->mode == GPIO_MODE_PWM) && (offset == pwmInfo[i].start)) {
                pwmInfo[i].enabled = true;
                pwmInfo[i].start = offset + DMA_FREQUENCY * pwmInfo[i].gpio->pwmPeriod / 1000.0f;
                pwmInfo[i].end = offset + DMA_FREQUENCY * pwmInfo[i].gpio->pwmWidth / 1000.0f;
                dmaCb[cbIndex].transferInfo = (0x01 << 26) | (0x01 << 3);
                dmaCb[cbIndex].srcAddress = getMemoryAddress(&bitMask[i]);
                dmaCb[cbIndex].dstAddress = getPeripheralAddress(pwmInfo[i].gpio->set);
                dmaCb[cbIndex].nextCbAddress = getMemoryAddress(&dmaCb[cbIndex + 1]);
                dmaCb[cbIndex].transferLen = sizeof(uint32_t);
                dmaCb[cbIndex].stride = 0;
                cbIndex++;
            } else if ((offset == pwmInfo[i].end) && pwmInfo[i].enabled) {
                dmaCb[cbIndex].transferInfo = (0x01 << 26) | (0x01 << 3);
                dmaCb[cbIndex].srcAddress = getMemoryAddress(&bitMask[i]);
                dmaCb[cbIndex].dstAddress = getPeripheralAddress(pwmInfo[i].gpio->clr);
                dmaCb[cbIndex].nextCbAddress = getMemoryAddress(&dmaCb[cbIndex + 1]);
                dmaCb[cbIndex].transferLen = sizeof(uint32_t);
                dmaCb[cbIndex].stride = 0;
                cbIndex++;
            }
            if (cbIndex == DMA_BUFFER_SIZE) {
                cbAvailable = false;
                break;
            }
        }
        if (cbAvailable) {
            dmaCb[cbIndex].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
            dmaCb[cbIndex].srcAddress = getMemoryAddress(pwmData);
            dmaCb[cbIndex].dstAddress = getPeripheralAddress(&pwm->fifoIn);
            dmaCb[cbIndex].nextCbAddress = getMemoryAddress(&dmaCb[cbIndex + 1]);
            dmaCb[cbIndex].transferLen = PWM_WRITES_PER_CYCLE * sizeof(uint32_t);
            dmaCb[cbIndex].stride = 0;
            cbIndex++;
        } else {
            cbIndex = lastComplete;
            break;
        }
        lastComplete = cbIndex;
        offset++;
    }
    dmaCb[cbIndex - 1].nextCbAddress = getMemoryAddress(dmaCb);

    *pwmData = 0x00000000;

    volatile DMARegisters *dma = (DMARegisters *)getPeripheral((dmaChannel < 15) ? DMA0_BASE_OFFSET + dmaChannel * 0x100 : DMA15_BASE_OFFSET);
    dma->ctlStatus = (0x01 << 31);
    usleep(1000);
    dma->ctlStatus = (0x01 << 2) | (0x01 << 1);
    dma->cbAddress = getMemoryAddress(dmaCb);
    dma->ctlStatus = (0xFF << 16) | 0x01;

    usleep(1000);

    while (pwmEnabled) {
        cbIndex = 0;
        while (cbIndex < DMA_BUFFER_SIZE) {
            cbAvailable = true;
            for (uint8_t i = 0; i < GPIO_COUNT; i++) {
                while (cbIndex == (dma->cbAddress - getMemoryAddress(dmaCb)) / sizeof(DMAControllBlock)) {
                    usleep(1);
                }
                if ((pwmInfo[i].gpio->mode == GPIO_MODE_PWM) && ((offset == pwmInfo[i].start) || !pwmInfo[i].enabled)) {
                    pwmInfo[i].enabled = true;
                    pwmInfo[i].start = offset + DMA_FREQUENCY * pwmInfo[i].gpio->pwmPeriod / 1000.0f;
                    pwmInfo[i].end = offset + DMA_FREQUENCY * pwmInfo[i].gpio->pwmWidth / 1000.0f;
                    dmaCb[cbIndex].transferInfo = (0x01 << 26) | (0x01 << 3);
                    dmaCb[cbIndex].srcAddress = getMemoryAddress(&bitMask[i]);
                    dmaCb[cbIndex].dstAddress = getPeripheralAddress(pwmInfo[i].gpio->set);
                    dmaCb[cbIndex].nextCbAddress = getMemoryAddress(&dmaCb[cbIndex + 1]);
                    dmaCb[cbIndex].transferLen = sizeof(uint32_t);
                    cbIndex++;
                } else if ((offset == pwmInfo[i].end) && pwmInfo[i].enabled) {
                    dmaCb[cbIndex].transferInfo = (0x01 << 26) | (0x01 << 3);
                    dmaCb[cbIndex].srcAddress = getMemoryAddress(&bitMask[i]);
                    dmaCb[cbIndex].dstAddress = getPeripheralAddress(pwmInfo[i].gpio->clr);
                    dmaCb[cbIndex].nextCbAddress = getMemoryAddress(&dmaCb[cbIndex + 1]);
                    dmaCb[cbIndex].transferLen = sizeof(uint32_t);
                    cbIndex++;
                } else if ((pwmInfo[i].gpio->mode != GPIO_MODE_PWM) && (offset == pwmInfo[i].start) && pwmInfo[i].enabled) {
                    pwmInfo[i].enabled = false;
                }
                if (cbIndex == DMA_BUFFER_SIZE) {
                    cbAvailable = false;
                    break;
                }
            }
            if (cbAvailable) {
                while (cbIndex == (dma->cbAddress - getMemoryAddress(dmaCb)) / sizeof(DMAControllBlock)) {
                    usleep(1);
                }
                dmaCb[cbIndex].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
                dmaCb[cbIndex].srcAddress = getMemoryAddress(pwmData);
                dmaCb[cbIndex].dstAddress = getPeripheralAddress(&pwm->fifoIn);
                dmaCb[cbIndex].nextCbAddress = getMemoryAddress(&dmaCb[cbIndex + 1]);
                dmaCb[cbIndex].transferLen = PWM_WRITES_PER_CYCLE * sizeof(uint32_t);
                cbIndex++;
            } else {
                cbIndex = lastComplete;
                break;
            }
            lastComplete = cbIndex;
            offset++;
        }
        dmaCb[cbIndex - 1].nextCbAddress = getMemoryAddress(dmaCb);
    }

    dmaCb[cbIndex - 1].nextCbAddress = 0x00000000;    
    while (dma->cbAddress != 0x00000000) {
        usleep(1);
    }

    dma->ctlStatus = (0x01 << 31);
    pwm->ctl = 0x00;

    delete [] pwmInfo;

    freeMemory();
    return NULL;
}

bool stop = false;

void sigIntHandler(int sigNum)
{
    stop = true;
}

int main(int argc, char** argv)
{
    signal(SIGINT, sigIntHandler);
    signal(SIGTSTP, sigIntHandler);
    try {
        GPIOController *gpio = &GPIOController::getInstance();
        gpio->setPwm(4, 20.0f, 1.0f); // Configure PWM on GPIO4, period = 20ms, pulse width = 1ms
        gpio->setMode(4, GPIO_MODE_PWM); // Turn on PWM on GPIO4 with selected configuration
        gpio->setMode(21, GPIO_MODE_OUT); // Use GPIO21 as an output
        gpio->set(21, true); // Set GPIO21 high
        usleep(1000000); // Wait 1s
        gpio->set(21, false); // Set GPIO21 low
        gpio->setPwm(4, 20.0f, 1.5f); // Change previously selected PWM pulse width to 1.5s
        usleep(1000000); // Wait 1s
        gpio->setPullUD(21, GPIO_PULL_UP); // Turn on pull-up on GPIO21
        gpio->setMode(21, GPIO_MODE_IN); // Use GPIO21 as an input
        cout << "INPUT: " << (gpio->get(21) ? "HIGH" : "LOW") << endl; // Print input level
    } catch (exception &error) {
        cout << "An error occurred, check your privileges" << endl;
        return 1;
    }

    return 0;
}
