/*
    rpi_gpio_service - Simple Raspberry Pi GPIO control service written in C++

    Copyright (c) 2019, Marcin Kondej
    All rights reserved.

    See https://github.com/markondej/rpi_gpio_service

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

#include "mailbox.h"
#include <bcm_host.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <iostream>
#include <thread>

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
#define GPIO_COUNT 28

#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_PWM 2
#define GPIO_MODE_UNKNOWN 3

#define GPIO_PULL_UP 0x02
#define GPIO_PULL_DOWN 0x01

#define GPIO0 0
#define GPIO1 1
#define GPIO2 2
#define GPIO3 3
#define GPIO4 4
#define GPIO5 5
#define GPIO6 6
#define GPIO7 7
#define GPIO8 8
#define GPIO9 9
#define GPIO10 10
#define GPIO11 11
#define GPIO12 12
#define GPIO13 13
#define GPIO14 14
#define GPIO15 15
#define GPIO16 16
#define GPIO17 17
#define GPIO18 18
#define GPIO19 19
#define GPIO20 20
#define GPIO21 21
#define GPIO22 22
#define GPIO23 23
#define GPIO24 24
#define GPIO25 25
#define GPIO26 26
#define GPIO27 27

#define SERVICE_PORT 5000
#define SERVICE_MAX_CONNECTIONS 10
#define RW_BUFFER_SIZE 1024

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
    volatile uint32_t *fnselReg;
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
        GPIOController(const GPIOController &source) = delete;
        GPIOController &operator=(const GPIOController &source) = delete;
        static GPIOController &getInstance();
        static void setDmaChannel(uint8_t dmaChannel);
        void setMode(uint8_t gpioNo, uint8_t mode);
        void setPullUd(uint8_t gpioNo, uint32_t type);
        void setPwm(uint8_t gpioNo, double period, double width);
        void set(uint8_t gpioNo, bool high);
        bool get(uint8_t gpioNo);
    private:
        GPIOController();
        GPIO *select(uint8_t gpioNo);
        static void pwmCallback();
        static uint32_t getMemoryAddress(volatile void *object);
        static uint32_t getPeripheralAddress(volatile void *object);
        static void *getPeripheral(uint32_t offset);
        static bool allocateMemory(uint32_t size);
        static void freeMemory();

        static void *peripherals;
        static GPIO *gpio;
        static bool pwmEnabled;
        static uint8_t dmaChannel;
        static uint32_t memSize, memAddress, memHandle;
        static volatile uint32_t *setReg, *clrReg;
        volatile uint32_t *levelReg;
        volatile GPIOPullUpDownRegisters *pud;
        static void *memAllocated;
        static int mBoxFd;

        std::thread *pwmThread;
};

void *GPIOController::peripherals = nullptr;
GPIO *GPIOController::gpio = nullptr;
bool GPIOController::pwmEnabled = false;
uint8_t GPIOController::dmaChannel = 0;
uint32_t GPIOController::memSize = 0;
uint32_t GPIOController::memAddress = 0x00000000;
uint32_t GPIOController::memHandle = 0;
volatile uint32_t *GPIOController::setReg = nullptr;
volatile uint32_t *GPIOController::clrReg = nullptr;
void *GPIOController::memAllocated = nullptr;
int GPIOController::mBoxFd = 0;

GPIOController::GPIOController()
{
    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw std::exception();
    }

    peripherals = mmap(nullptr, bcm_host_get_peripheral_size(), PROT_READ | PROT_WRITE, MAP_SHARED, memFd, bcm_host_get_peripheral_address());
    close(memFd);
    if (peripherals == MAP_FAILED) {
        throw std::exception();
    }

    setReg = (uint32_t *)getPeripheral(GPIO_SET0_OFFSET);
    clrReg = (uint32_t *)getPeripheral(GPIO_CLR0_OFFSET);
    levelReg = (uint32_t *)getPeripheral(GPIO_LEVEL0_OFFSET);
    pud = (GPIOPullUpDownRegisters *)getPeripheral(GPIO_PUDCTL_OFFSET);

    gpio = new GPIO[GPIO_COUNT];
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        gpio[i] = {
            i,
            (uint32_t *)getPeripheral(GPIO_FSEL_BASE_OFFSET + ((uint32_t)i * 3) / 30 * sizeof(uint32_t)),
            ((uint32_t)i * 3) % 30,
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
        pwmThread->join();
        delete pwmThread;
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
        throw std::exception();
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

void* GPIOController::getPeripheral(uint32_t offset) {
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

GPIO *GPIOController::select(uint8_t gpioNo)
{
    if ((gpioNo >= GPIO_COUNT) || (gpio[gpioNo].number != gpioNo)) {
        throw std::exception();
    }
    return &gpio[gpioNo];
}

void GPIOController::setMode(uint8_t gpioNo, uint8_t mode)
{
    GPIO *selected = select(gpioNo);
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
            throw std::exception();
    }
    uint32_t fnsel = *selected->fnselReg & ((0xFFFFFFF8 << selected->fnselBit) | (0xFFFFFFFF >> (32 - selected->fnselBit)));
    *selected->fnselReg = fnsel | (func << selected->fnselBit);
    if (mode == GPIO_MODE_PWM) {
        if (!pwmEnabled) {
            pwmEnabled = true;
            pwmThread = new std::thread(GPIOController::pwmCallback);
        }
    } else {
        bool stop = true;
        for (uint8_t i = 0; i < GPIO_COUNT; i++) {
            if ((selected != &gpio[i]) && (gpio[i].mode == GPIO_MODE_PWM)) {
                stop = false;
                break;
            }
        }
        if (stop) {
            pwmEnabled = false;
            pwmThread->join();
            delete pwmThread;
        }
    }
}

void GPIOController::setPullUd(uint8_t gpioNo, uint32_t type) {
    GPIO *selected = select(gpioNo);
    pud->ctl = type;
    usleep(100);
    pud->clock0 = 0x01 << selected->number;
    usleep(100);
    pud->ctl = 0x00000000;
    pud->clock0 = 0x00000000;
}

void GPIOController::setPwm(uint8_t gpioNo, double period, double width)
{
    if ((period > 0.0f) && (width >= 0.0f) && (width <= period)) {
        GPIO *selected = select(gpioNo);
        selected->pwmPeriod = period;
        selected->pwmWidth = width;
    }
}

void GPIOController::set(uint8_t gpioNo, bool high)
{
    GPIO *selected = select(gpioNo);
    if (selected->mode != GPIO_MODE_PWM) {
        volatile uint32_t *reg = high ? setReg : clrReg;
        *reg = 0x01 << selected->number;
    }
}

bool GPIOController::get(uint8_t gpioNo)
{
    GPIO *selected = select(gpioNo);
    if (selected->mode != GPIO_MODE_PWM) {
        return (bool)(*levelReg & (0x01 << selected->number));
    }
    return false;
}

void GPIOController::pwmCallback()
{
    if (allocateMemory(DMA_BUFFER_SIZE * sizeof(DMAControllBlock) + DMA_BUFFER_SIZE * sizeof(uint32_t))) {
        PWM *pwmInfo = new PWM[GPIO_COUNT];
        memset(pwmInfo, 0, sizeof(PWM) * GPIO_COUNT);
        for (uint8_t i = 0; i < GPIO_COUNT; i++) {
            pwmInfo[i].gpio = &gpio[i];
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

        volatile DMAControllBlock *dmaCb = (DMAControllBlock *)memAllocated;
        volatile uint32_t *bitMask = (uint32_t *)((uint32_t)dmaCb + DMA_BUFFER_SIZE * sizeof(DMAControllBlock));

        uint32_t cbOffset = 0;
        memset((void *)dmaCb, 0, sizeof(DMAControllBlock) * DMA_BUFFER_SIZE);
        memset((void *)bitMask, 0, sizeof(uint32_t) * DMA_BUFFER_SIZE);
        dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
        dmaCb[cbOffset].srcAddress = getMemoryAddress(&bitMask[cbOffset]);
        dmaCb[cbOffset].dstAddress = getPeripheralAddress(&pwm->fifoIn);
        dmaCb[cbOffset].nextCbAddress = getMemoryAddress(&dmaCb[cbOffset + 1]);
        dmaCb[cbOffset].transferLen = 8 * PWM_WRITES_PER_CYCLE * sizeof(uint32_t);
        cbOffset++;

        bool cbAvailable;
        uint64_t offset = 0;
        uint32_t bitMaskSetClr[2], lastComplete = cbOffset;
        while (cbOffset < DMA_BUFFER_SIZE) {
            memset(bitMaskSetClr, 0, sizeof(uint32_t) * 2);
            for (uint8_t i = 0; i < GPIO_COUNT; i++) {
                if ((pwmInfo[i].gpio->mode == GPIO_MODE_PWM) && (offset == pwmInfo[i].start)) {
                    pwmInfo[i].enabled = true;
                    pwmInfo[i].start = offset + DMA_FREQUENCY * pwmInfo[i].gpio->pwmPeriod / 1000.0f;
                    pwmInfo[i].end = offset + DMA_FREQUENCY * pwmInfo[i].gpio->pwmWidth / 1000.0f;
                    bitMaskSetClr[(pwmInfo[i].end != offset) ? 1 : 0] |= 0x01 << pwmInfo[i].gpio->number;
                } else if ((offset == pwmInfo[i].end) && pwmInfo[i].enabled && (pwmInfo[i].end != pwmInfo[i].start)) {
                    bitMaskSetClr[0] |= 0x01 << pwmInfo[i].gpio->number;
                }
            }
            cbAvailable = true;
            for (uint8_t i = 0; i < 2; i++) {
                if (bitMaskSetClr[i]) {
                    bitMask[cbOffset] = bitMaskSetClr[i];
                    dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x01 << 3);
                    dmaCb[cbOffset].srcAddress = getMemoryAddress(&bitMask[cbOffset]);
                    dmaCb[cbOffset].dstAddress = getPeripheralAddress((i > 0) ? setReg : clrReg);
                    dmaCb[cbOffset].nextCbAddress = getMemoryAddress(&dmaCb[cbOffset + 1]);
                    dmaCb[cbOffset].transferLen = sizeof(uint32_t);
                    cbOffset++;
                    if (cbOffset == DMA_BUFFER_SIZE) {
                        cbAvailable = false;
                        break;
                    }
                }
            }
            if (cbAvailable) {
                dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
                dmaCb[cbOffset].srcAddress = getMemoryAddress(&bitMask[cbOffset]);
                dmaCb[cbOffset].dstAddress = getPeripheralAddress(&pwm->fifoIn);
                dmaCb[cbOffset].nextCbAddress = getMemoryAddress(&dmaCb[cbOffset + 1]);
                dmaCb[cbOffset].transferLen = PWM_WRITES_PER_CYCLE * sizeof(uint32_t);
                cbOffset++;
            } else {
                cbOffset = lastComplete;
                break;
            }
            lastComplete = cbOffset;
            offset++;
        }
        dmaCb[cbOffset - 1].nextCbAddress = getMemoryAddress(dmaCb);

        volatile DMARegisters *dma = (DMARegisters *)getPeripheral((dmaChannel < 15) ? DMA0_BASE_OFFSET + dmaChannel * 0x100 : DMA15_BASE_OFFSET);
        dma->ctlStatus = (0x01 << 31);
        usleep(1000);
        dma->ctlStatus = (0x01 << 2) | (0x01 << 1);
        dma->cbAddress = getMemoryAddress(dmaCb);
        dma->ctlStatus = (0xFF << 16) | 0x01;

        usleep(DMA_BUFFER_SIZE * 250000 / DMA_FREQUENCY);

        while (pwmEnabled) {
            cbOffset = 0;
            while (cbOffset < DMA_BUFFER_SIZE) {
                memset(bitMaskSetClr, 0, sizeof(uint32_t) * 2);
                for (uint8_t i = 0; i < GPIO_COUNT; i++) {
                    if ((pwmInfo[i].gpio->mode == GPIO_MODE_PWM) && ((offset == pwmInfo[i].start) || !pwmInfo[i].enabled)) {
                        pwmInfo[i].enabled = true;
                        pwmInfo[i].start = offset + DMA_FREQUENCY * pwmInfo[i].gpio->pwmPeriod / 1000.0f;
                        pwmInfo[i].end = offset + DMA_FREQUENCY * pwmInfo[i].gpio->pwmWidth / 1000.0f;
                        bitMaskSetClr[(pwmInfo[i].end != offset) ? 1 : 0] |= 0x01 << pwmInfo[i].gpio->number;
                    } else if ((offset == pwmInfo[i].end) && pwmInfo[i].enabled && (pwmInfo[i].end != pwmInfo[i].start)) {
                        bitMaskSetClr[0] |= 0x01 << pwmInfo[i].gpio->number;
                    } else if ((pwmInfo[i].gpio->mode != GPIO_MODE_PWM) && (offset == pwmInfo[i].start) && pwmInfo[i].enabled) {
                        pwmInfo[i].enabled = false;
                    }
                }
                cbAvailable = true;
                for (uint8_t i = 0; i < 2; i++) {
                    if (bitMaskSetClr[i]) {
                        while (cbOffset == (dma->cbAddress - getMemoryAddress(dmaCb)) / sizeof(DMAControllBlock)) {
                            usleep(1000);
                        }
                        bitMask[cbOffset] = bitMaskSetClr[i];
                        dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x01 << 3);
                        dmaCb[cbOffset].srcAddress = getMemoryAddress(&bitMask[cbOffset]);
                        dmaCb[cbOffset].dstAddress = getPeripheralAddress((i > 0) ? setReg : clrReg);
                        dmaCb[cbOffset].nextCbAddress = getMemoryAddress(&dmaCb[cbOffset + 1]);
                        dmaCb[cbOffset].transferLen = sizeof(uint32_t);
                        cbOffset++;
                        if (cbOffset == DMA_BUFFER_SIZE) {
                            cbAvailable = false;
                            break;
                        }
                    }
                }
                if (cbAvailable) {
                    while (cbOffset == (dma->cbAddress - getMemoryAddress(dmaCb)) / sizeof(DMAControllBlock)) {
                        usleep(1000);
                    }
                    bitMask[cbOffset] = 0x00000000;
                    dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
                    dmaCb[cbOffset].srcAddress = getMemoryAddress(&bitMask[cbOffset]);
                    dmaCb[cbOffset].dstAddress = getPeripheralAddress(&pwm->fifoIn);
                    dmaCb[cbOffset].nextCbAddress = getMemoryAddress(&dmaCb[cbOffset + 1]);
                    dmaCb[cbOffset].transferLen = PWM_WRITES_PER_CYCLE * sizeof(uint32_t);
                    cbOffset++;
                } else {
                    cbOffset = lastComplete;
                    break;
                }
                lastComplete = cbOffset;
                offset++;
            }
            dmaCb[cbOffset - 1].nextCbAddress = getMemoryAddress(dmaCb);
        }

        dmaCb[cbOffset - 1].nextCbAddress = 0x00000000;
        while (dma->cbAddress != 0x00000000) {
            usleep(1000);
        }

        dma->ctlStatus = (0x01 << 31);
        pwm->ctl = 0x00;

        delete[] pwmInfo;

        freeMemory();
    }
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
        int socketFd, acceptedFd, enable = 1;
        struct sockaddr_in servAddr;

        char readBuff[RW_BUFFER_SIZE];
        char sendBuff[RW_BUFFER_SIZE];

        if ((socketFd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1) {
            throw std::exception();
        }

        if (setsockopt(socketFd, SOL_SOCKET, SO_REUSEADDR, (char *)&enable, sizeof(enable)) == -1) {
            throw std::exception();
        }

        memset(&servAddr, 0, sizeof(servAddr));
        servAddr.sin_family = AF_INET;
        servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        servAddr.sin_port = htons(SERVICE_PORT);

        if (bind(socketFd, (struct sockaddr*)&servAddr, sizeof(servAddr)) == -1) {
            close(socketFd);
            throw std::exception();
        }

        if (listen(socketFd, SERVICE_MAX_CONNECTIONS) == -1) {
            close(socketFd);
            throw std::exception();
        }

        GPIOController *gpio = &GPIOController::getInstance();
        gpio->setPwm(GPIO4, 20.0f, 1.0f);
        gpio->setMode(GPIO4, GPIO_MODE_PWM);

        while(!stop) {
            if ((acceptedFd = accept(socketFd, (struct sockaddr*)nullptr, nullptr)) == -1) {
                usleep(1000);
                continue;
            }
            memset(readBuff, 0, sizeof(readBuff));
            memset(sendBuff, 0, sizeof(sendBuff));
            if (read(acceptedFd, readBuff, sizeof(readBuff)) == -1) {
                close(acceptedFd);
                continue;
            }
            if (strcmp(readBuff, "1\r\n") == 0) {
                sprintf(sendBuff, "OK:1\r\n");
                gpio->setPwm(GPIO4, 20.0f, 2.0f);
                std::cout << "Level set: 1" << std::endl;
            } else if (strcmp(readBuff, "0\r\n") == 0) {
                sprintf(sendBuff, "OK:0\r\n");
                gpio->setPwm(GPIO4, 20.0f, 1.0f);
                std::cout << "Level set: 0" << std::endl;
            } else {
                sprintf(sendBuff, "ERROR:VALUE UNKNOWN\r\n");
                std::cout << "Received unknown value" << std::endl;
            }
            write(acceptedFd, sendBuff, strlen(sendBuff));
            shutdown(acceptedFd, SHUT_RDWR);
            close(acceptedFd);
        }

        gpio->setMode(GPIO4, GPIO_MODE_OUT);
        gpio->set(GPIO4, false);

        close(socketFd);
    } catch (std::exception &error) {
        std::cout << "An error occurred, check your privileges" << std::endl;
        return 1;
    }

    return 0;
}
