/*
    rpi_gpio_service - Raspberry Pi GPIO RESTful API written in C++

    Copyright (c) 2021, Marcin Kondej
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
#include "httplib/httplib.h"
#include "nlohmann/json.hpp"
#include <bcm_host.h>
#include <sys/mman.h>
#include <fcntl.h>

#define PERIPHERALS_PHYS_BASE 0x7e000000
#define BCM2835_PERIPHERALS_VIRT_BASE 0x20000000
#define BCM2838_PERIPHERALS_VIRT_BASE 0xfe000000
#define GPIO_FSEL_BASE_OFFSET 0x00200000
#define GPIO_SET0_OFFSET 0x0020001c
#define GPIO_CLR0_OFFSET 0x00200028
#define GPIO_LEVEL0_OFFSET 0x00200034
#define GPIO_PUDCTL_OFFSET 0x00200094
#define DMA0_BASE_OFFSET 0x00007000
#define DMA15_BASE_OFFSET 0x00e05000
#define PWMCLK_BASE_OFFSET 0x001010a0
#define PWM_BASE_OFFSET 0x0020c000

#define BCM2835_MEM_FLAG 0x0c
#define BCM2838_MEM_FLAG 0x04

#define BCM2835_PLLD_FREQ 500
#define BCM2838_PLLD_FREQ 750

#define PWM_CHANNEL_RANGE 32
#define DMA_BUFFER_SIZE 1024
#define DMA_FREQUENCY 100000
#define PWM_WRITES_PER_CYCLE 4
#define PAGE_SIZE 4096
#define GPIO_COUNT 28

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

#define DEFAULT_SERVICE_PORT 8080
#define DEFAULT_SERVICE_ADDRESS "0.0.0.0"

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

enum class GPIOMode { In, Out, PWM };

enum class GPIOResistor { PullUp, PullDown };

struct GPIO {
    volatile uint32_t *fnselReg;
    uint32_t fnselBit;
    GPIOMode mode;
    float pwmPeriod, pwmWidth;
    std::mutex access;
};

struct PWM {
    unsigned long long start, end;
    bool enabled;
};

class Peripherals
{
    public:
        virtual ~Peripherals();
        Peripherals(const Peripherals &) = delete;
        Peripherals(Peripherals &&) = delete;
        Peripherals &operator=(const Peripherals &) = delete;
        static Peripherals &GetInstance();
        uintptr_t GetPhysicalAddress(volatile void *object) const;
        uintptr_t GetVirtualAddress(uintptr_t offset) const;
        static uintptr_t GetVirtualBaseAddress();
        static float GetClockFrequency();
    private:
        Peripherals();
        unsigned GetSize();

        void *peripherals;
};

Peripherals::Peripherals()
{
    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw std::runtime_error("Cannot open /dev/mem file (permission denied)");
    }

    peripherals = mmap(nullptr, GetSize(), PROT_READ | PROT_WRITE, MAP_SHARED, memFd, GetVirtualBaseAddress());
    close(memFd);
    if (peripherals == MAP_FAILED) {
        throw std::runtime_error("Cannot obtain access to peripherals (mmap error)");
    }
}

Peripherals::~Peripherals()
{
    munmap(peripherals, GetSize());
}

Peripherals &Peripherals::GetInstance()
{
    static Peripherals instance;
    return instance;
}

uintptr_t Peripherals::GetPhysicalAddress(volatile void *object) const
{
    return PERIPHERALS_PHYS_BASE + (reinterpret_cast<uintptr_t>(object) - reinterpret_cast<uintptr_t>(peripherals));
}

uintptr_t Peripherals::GetVirtualAddress(uintptr_t offset) const
{
    return reinterpret_cast<uintptr_t>(peripherals) + offset;
}

uintptr_t Peripherals::GetVirtualBaseAddress()
{
    return (bcm_host_get_peripheral_size() == BCM2838_PERIPHERALS_VIRT_BASE) ? BCM2838_PERIPHERALS_VIRT_BASE : bcm_host_get_peripheral_address();
}

float Peripherals::GetClockFrequency()
{
    return (GetVirtualBaseAddress() == BCM2838_PERIPHERALS_VIRT_BASE) ? BCM2838_PLLD_FREQ : BCM2835_PLLD_FREQ;
}

unsigned Peripherals::GetSize()
{
    unsigned size = bcm_host_get_peripheral_size();
    if (size == BCM2838_PERIPHERALS_VIRT_BASE) {
        size = 0x01000000;
    }
    return size;
}

class AllocatedMemory
{
    public:
        AllocatedMemory(unsigned size);
        virtual ~AllocatedMemory();
        AllocatedMemory(const AllocatedMemory &) = delete;
        AllocatedMemory(AllocatedMemory &&) = delete;
        AllocatedMemory &operator=(const AllocatedMemory &) = delete;
        uintptr_t GetPhysicalAddress(volatile void *object) const;
        uintptr_t GetBaseAddress() const;
    private:
        unsigned memSize, memHandle;
        uintptr_t memAddress;
        void *memAllocated;
        int mBoxFd;
};

AllocatedMemory::AllocatedMemory(unsigned size)
{
    mBoxFd = mbox_open();
    memSize = size;
    if (memSize % PAGE_SIZE) {
        memSize = (memSize / PAGE_SIZE + 1) * PAGE_SIZE;
    }
    memHandle = mem_alloc(mBoxFd, size, PAGE_SIZE, (Peripherals::GetVirtualBaseAddress() == BCM2835_PERIPHERALS_VIRT_BASE) ? BCM2835_MEM_FLAG : BCM2838_MEM_FLAG);
    if (!memHandle) {
        mbox_close(mBoxFd);
        memSize = 0;
        throw std::runtime_error("Cannot allocate memory (" + std::to_string(size) + "bytes");
    }
    memAddress = mem_lock(mBoxFd, memHandle);
    memAllocated = mapmem(memAddress & ~0xc0000000, memSize);
}

AllocatedMemory::~AllocatedMemory()
{
    unmapmem(memAllocated, memSize);
    mem_unlock(mBoxFd, memHandle);
    mem_free(mBoxFd, memHandle);

    mbox_close(mBoxFd);
    memSize = 0;
}

uintptr_t AllocatedMemory::GetPhysicalAddress(volatile void *object) const
{
    return (memSize) ? memAddress + (reinterpret_cast<uintptr_t>(object) - reinterpret_cast<uintptr_t>(memAllocated)) : 0x00000000;
}

uintptr_t AllocatedMemory::GetBaseAddress() const
{
    return reinterpret_cast<uintptr_t>(memAllocated);
}

class GPIOController
{
    public:
        virtual ~GPIOController();
        GPIOController(const GPIOController &) = delete;
        GPIOController(GPIOController &&) = delete;
        GPIOController &operator=(const GPIOController &) = delete;
        static GPIOController &GetInstance();
        void SetDMAChannel(unsigned channel);
        void SetMode(unsigned gpio, GPIOMode mode);
        void SetResistor(unsigned gpio, GPIOResistor resistor);
        void SetPWM(unsigned gpio, float period, float width);
        void Set(unsigned gpio, bool active);
        bool Get(unsigned gpio);
    private:
        GPIOController();
        GPIO &Select(unsigned gpio);
        static void PWMCallback(GPIOController *instance);

        GPIO gpio[GPIO_COUNT];
        std::atomic_uint pwmOutputs, dmaChannel;
        volatile uint32_t *setReg, *clrReg;
        volatile uint32_t *levelReg;
        volatile GPIOPullUpDownRegisters *pud;

        std::thread *pwmThread;
};

GPIOController::GPIOController()
    : pwmOutputs(0), dmaChannel(0)
{
    Peripherals &peripherals = Peripherals::GetInstance();

    setReg = reinterpret_cast<uint32_t *>(peripherals.GetVirtualAddress(GPIO_SET0_OFFSET));
    clrReg = reinterpret_cast<uint32_t *>(peripherals.GetVirtualAddress(GPIO_CLR0_OFFSET));
    levelReg = reinterpret_cast<uint32_t *>(peripherals.GetVirtualAddress(GPIO_LEVEL0_OFFSET));
    pud = reinterpret_cast<GPIOPullUpDownRegisters *>(peripherals.GetVirtualAddress(GPIO_PUDCTL_OFFSET));

    for (unsigned i = 0; i < GPIO_COUNT; i++) {
        gpio[i].fnselReg = reinterpret_cast<uint32_t *>(peripherals.GetVirtualAddress(GPIO_FSEL_BASE_OFFSET + i * 3 / 30 * sizeof(uint32_t)));
        gpio[i].fnselBit = (i * 3) % 30;
        gpio[i].mode = GPIOMode::Out;
        gpio[i].pwmPeriod = 20.f;
        gpio[i].pwmWidth = 1.f;
    }
}

GPIOController::~GPIOController()
{
    if (pwmOutputs) {
        pwmOutputs = 0;
        pwmThread->join();
        delete pwmThread;
    }
}

GPIOController &GPIOController::GetInstance()
{
    static GPIOController instance;
    return instance;
}

void GPIOController::SetDMAChannel(unsigned channel)
{
    if (channel > 15) {
        throw std::runtime_error("Selected DMA channel is not supported");
    }
    dmaChannel = channel;
}

GPIO &GPIOController::Select(unsigned gpio)
{
    if (gpio >= GPIO_COUNT) {
        throw std::runtime_error("Selected GPIO is not supported");
    }
    return this->gpio[gpio];
}

void GPIOController::SetMode(unsigned gpio, GPIOMode mode)
{
    GPIO &selected = Select(gpio);
    uint8_t func;
    switch (mode) {
        case GPIOMode::In:
            func = 0x00;
            break;
        default:
            func = 0x01;
    }
    bool pwmDisable = false;
    {
        std::lock_guard<std::mutex> lock(selected.access);
        pwmDisable = (selected.mode == GPIOMode::PWM) && (mode != GPIOMode::PWM);
        uint32_t fnsel = *selected.fnselReg & ((0xfffffff8 << selected.fnselBit) | (0xffffffff >> (32 - selected.fnselBit)));
        *selected.fnselReg = fnsel | (func << selected.fnselBit);
        selected.mode = mode;
    }
    if ((mode == GPIOMode::PWM) && (selected.mode != GPIOMode::PWM)) {
        if (!(pwmOutputs++)) {
            pwmThread = new std::thread(GPIOController::PWMCallback, this);
        }
    } else if (pwmDisable) {
        if (!(--pwmOutputs)) {
            pwmThread->join();
            delete pwmThread;
        }
    }
}

void GPIOController::SetResistor(unsigned gpio, GPIOResistor resistor)
{
    Select(gpio);
    switch (resistor) {
        case GPIOResistor::PullDown:
            pud->ctl = 0x01;
            break;
        default:
            pud->ctl = 0x02;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    pud->clock0 = 0x01 << gpio;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    pud->ctl = 0x00000000;
    pud->clock0 = 0x00000000;
}

void GPIOController::SetPWM(unsigned gpio, float period, float width)
{
    if ((period > 0.f) && (width >= 0.f) && (width <= period)) {
        GPIO &selected = Select(gpio);
        std::lock_guard<std::mutex> lock(selected.access);
        selected.pwmPeriod = period;
        selected.pwmWidth = width;
    }
}

void GPIOController::Set(unsigned gpio, bool active)
{
    GPIO &selected = Select(gpio);
    std::lock_guard<std::mutex> lock(selected.access);
    if (selected.mode != GPIOMode::PWM) {
        volatile uint32_t *reg = active ? setReg : clrReg;
        *reg = 0x01 << gpio;
    }
}

bool GPIOController::Get(unsigned gpio)
{
    GPIO &selected = Select(gpio);
    std::lock_guard<std::mutex> lock(selected.access);
    if (selected.mode != GPIOMode::PWM) {
        return (bool)(*levelReg & (0x01 << gpio));
    }
    return false;
}

void GPIOController::PWMCallback(GPIOController *instance)
{
    AllocatedMemory allocated(DMA_BUFFER_SIZE * sizeof(DMAControllBlock) + DMA_BUFFER_SIZE * sizeof(uint32_t));

    Peripherals &peripherals = Peripherals::GetInstance();

    volatile ClockRegisters *pwmClk = reinterpret_cast<ClockRegisters *>(peripherals.GetVirtualAddress(PWMCLK_BASE_OFFSET));
    pwmClk->ctl = (0x5a << 24) | 0x06;
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    pwmClk->div = (0x5a << 24) | static_cast<uint32_t>(Peripherals::GetClockFrequency() * (0x01 << 12) / (PWM_CHANNEL_RANGE * PWM_WRITES_PER_CYCLE * DMA_FREQUENCY / 1000000.f));
    pwmClk->ctl = (0x5a << 24) | (0x01 << 4) | 0x06;

    volatile PWMRegisters *pwm = reinterpret_cast<PWMRegisters *>(peripherals.GetVirtualAddress(PWM_BASE_OFFSET));
    pwm->ctl = 0x00;
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    pwm->status = 0x01fc;
    pwm->ctl = (0x01 << 6);
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    pwm->chn1Range = PWM_CHANNEL_RANGE;
    pwm->dmaConf = (0x01 << 31) | 0x0707;
    pwm->ctl = (0x01 << 5) | (0x01 << 2) | 0x01;

    volatile DMAControllBlock *dmaCb = reinterpret_cast<DMAControllBlock *>(allocated.GetBaseAddress());
    volatile uint32_t *bitMask = reinterpret_cast<uint32_t *>(reinterpret_cast<uintptr_t>(dmaCb) + DMA_BUFFER_SIZE * sizeof(DMAControllBlock));

    unsigned cbOffset = 0;
    std::memset(const_cast<DMAControllBlock *>(dmaCb), 0, sizeof(DMAControllBlock) * DMA_BUFFER_SIZE);
    std::memset(const_cast<uint32_t *>(bitMask), 0, sizeof(uint32_t) * DMA_BUFFER_SIZE);
    dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
    dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(&bitMask[cbOffset]);
    dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress(&pwm->fifoIn);
    dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
    dmaCb[cbOffset].transferLen = 8 * PWM_WRITES_PER_CYCLE * sizeof(uint32_t);
    cbOffset++;

    PWM pwmInfo[GPIO_COUNT];
    std::memset(pwmInfo, 0, sizeof(PWM) * GPIO_COUNT);

    bool cbAvailable;
    uint32_t bitMaskSetClr[2];
    unsigned long long offset = 0;
    unsigned lastComplete = cbOffset;
    while (cbOffset < DMA_BUFFER_SIZE) {
        memset(bitMaskSetClr, 0, sizeof(uint32_t) * 2);
        for (unsigned i = 0; i < GPIO_COUNT; i++) {
            std::lock_guard<std::mutex> lock(instance->gpio[i].access);
            if ((instance->gpio[i].mode == GPIOMode::PWM) && (offset == pwmInfo[i].start)) {
                pwmInfo[i].enabled = true;
                pwmInfo[i].start = static_cast<unsigned long long>(offset + DMA_FREQUENCY * static_cast<double>(instance->gpio[i].pwmPeriod) / 1000.);
                pwmInfo[i].end = static_cast<unsigned long long>(offset + DMA_FREQUENCY * static_cast<double>(instance->gpio[i].pwmWidth) / 1000.);
                bitMaskSetClr[(pwmInfo[i].end != offset) ? 1 : 0] |= 0x01 << i;
            } else if ((offset == pwmInfo[i].end) && pwmInfo[i].enabled && (pwmInfo[i].end != pwmInfo[i].start)) {
                bitMaskSetClr[0] |= 0x01 << i;
            }
        }
        cbAvailable = true;
        for (unsigned i = 0; i < 2; i++) {
            if (bitMaskSetClr[i]) {
                bitMask[cbOffset] = bitMaskSetClr[i];
                dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x01 << 3);
                dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(&bitMask[cbOffset]);
                dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress((i > 0) ? instance->setReg : instance->clrReg);
                dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
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
            dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(&bitMask[cbOffset]);
            dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress(&pwm->fifoIn);
            dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
            dmaCb[cbOffset].transferLen = PWM_WRITES_PER_CYCLE * sizeof(uint32_t);
            cbOffset++;
        } else {
            cbOffset = lastComplete;
            break;
        }
        lastComplete = cbOffset;
        offset++;
    }
    dmaCb[cbOffset - 1].nextCbAddress = allocated.GetPhysicalAddress(dmaCb);

    unsigned dmaChannel = instance->dmaChannel;
    volatile DMARegisters *dma = reinterpret_cast<DMARegisters *>(peripherals.GetVirtualAddress((dmaChannel < 15) ? DMA0_BASE_OFFSET + dmaChannel * 0x100 : DMA15_BASE_OFFSET));
    dma->ctlStatus = (0x01 << 31);
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    dma->ctlStatus = (0x01 << 2) | (0x01 << 1);
    dma->cbAddress = allocated.GetPhysicalAddress(dmaCb);
    dma->ctlStatus = (0xff << 16) | 0x01;

    std::this_thread::sleep_for(std::chrono::microseconds(DMA_BUFFER_SIZE * 250000 / DMA_FREQUENCY));

    while (instance->pwmOutputs) {
        cbOffset = 0;
        while (cbOffset < DMA_BUFFER_SIZE) {
            std::memset(bitMaskSetClr, 0, sizeof(uint32_t) * 2);
            for (unsigned i = 0; i < GPIO_COUNT; i++) {
                std::lock_guard<std::mutex> lock(instance->gpio[i].access);
                if ((instance->gpio[i].mode == GPIOMode::PWM) && ((offset == pwmInfo[i].start) || !pwmInfo[i].enabled)) {
                    pwmInfo[i].enabled = true;
                    pwmInfo[i].start = static_cast<unsigned long long>(offset + DMA_FREQUENCY * static_cast<double>(instance->gpio[i].pwmPeriod) / 1000.);
                    pwmInfo[i].end = static_cast<unsigned long long>(offset + DMA_FREQUENCY * static_cast<double>(instance->gpio[i].pwmWidth) / 1000.);
                    bitMaskSetClr[(pwmInfo[i].end != offset) ? 1 : 0] |= 0x01 << i;
                } else if ((offset == pwmInfo[i].end) && pwmInfo[i].enabled && (pwmInfo[i].end != pwmInfo[i].start)) {
                    bitMaskSetClr[0] |= 0x01 << i;
                } else if ((instance->gpio[i].mode != GPIOMode::PWM) && (offset == pwmInfo[i].start) && pwmInfo[i].enabled) {
                    pwmInfo[i].enabled = false;
                }
            }
            cbAvailable = true;
            for (unsigned i = 0; i < 2; i++) {
                if (bitMaskSetClr[i]) {
                    while (cbOffset == (dma->cbAddress - allocated.GetPhysicalAddress(dmaCb)) / sizeof(DMAControllBlock)) {
                        std::this_thread::sleep_for(std::chrono::microseconds(1000));
                    }
                    bitMask[cbOffset] = bitMaskSetClr[i];
                    dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x01 << 3);
                    dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(&bitMask[cbOffset]);
                    dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress((i > 0) ? instance->setReg : instance->clrReg);
                    dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
                    dmaCb[cbOffset].transferLen = sizeof(uint32_t);
                    cbOffset++;
                    if (cbOffset == DMA_BUFFER_SIZE) {
                        cbAvailable = false;
                        break;
                    }
                }
            }
            if (cbAvailable) {
                while (cbOffset == (dma->cbAddress - allocated.GetPhysicalAddress(dmaCb)) / sizeof(DMAControllBlock)) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                }
                bitMask[cbOffset] = 0x00000000;
                dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
                dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(&bitMask[cbOffset]);
                dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress(&pwm->fifoIn);
                dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
                dmaCb[cbOffset].transferLen = PWM_WRITES_PER_CYCLE * sizeof(uint32_t);
                cbOffset++;
            } else {
                cbOffset = lastComplete;
                break;
            }
            lastComplete = cbOffset;
            offset++;
        }
        dmaCb[cbOffset - 1].nextCbAddress = allocated.GetPhysicalAddress(dmaCb);
    }

    dmaCb[cbOffset - 1].nextCbAddress = 0x00000000;
    while (dma->cbAddress != 0x00000000) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    dma->ctlStatus = (0x01 << 31);
    pwm->ctl = 0x00;
}

httplib::Server server;

nlohmann::json parseJsonObject(const std::string &data) {
    try {
        nlohmann::json json = nlohmann::json::parse(data);
        if (!json.is_object()) {
            throw std::exception();
        }
        return json;
    } catch (...) {
        throw std::runtime_error("Syntax error, cannot parse object");
    }
}

template <class T>
T loadJsonProperty(const nlohmann::json &json, const char *property)
{
    try {
        if (json.is_object() && json.contains(property) && !json[property].is_null()) {
            return json[property].get<T>();
        }
    } catch (...) {
    }
    throw std::runtime_error("'" + std::string(property) + "' value is missing or invalid");
}

void handleResource(const httplib::Request &request, httplib::Response &response, httplib::Server::Handler handler)
{
    try {
        handler(request, response);
    } catch (std::exception &catched) {
        nlohmann::json jsonError;
        jsonError["error"] = catched.what();
        if (response.status == -1) {
            response.status = 500;
        }
        response.set_content(jsonError.dump(), "application/json");
    }
}

void sigIntHandler(int sigNum)
{
    if (server.is_running()) {
        server.stop();
    }
}

int main(int argc, char** argv)
{
    std::string address = DEFAULT_SERVICE_ADDRESS;
    int opt, port = DEFAULT_SERVICE_PORT;
    while ((opt = getopt(argc, argv, "a:p:")) != -1) {
        switch (opt) {
        case 'a':
            address = optarg;
            break;
        case 'p':
            port = std::atoi(optarg);
            break;
        }
    }

    signal(SIGINT, sigIntHandler);
    signal(SIGTSTP, sigIntHandler);

    GPIOController &gpio = GPIOController::GetInstance();

    server.Get("/gpio([0-9]{1,2})", [&](const httplib::Request &request, httplib::Response &response) {
        handleResource(request, response, [&](const httplib::Request &request, httplib::Response &response) {
            nlohmann::json json;
            json["active"] = gpio.Get(std::stoi(request.matches[1]));
            response.set_content(json.dump(), "application/json");
        });
    });

    server.Put("/gpio([0-9]{1,2})", [&](const httplib::Request &request, httplib::Response &response) {
        handleResource(request, response, [&](const httplib::Request &request, httplib::Response &response) {
            bool active = loadJsonProperty<bool>(parseJsonObject(request.body), "active");
            gpio.Set(std::stoi(request.matches[1]), active);
        });
    });

    server.Put("/gpio([0-9]{1,2})/mode", [&](const httplib::Request &request, httplib::Response &response) {
        handleResource(request, response, [&](const httplib::Request &request, httplib::Response &response) {
            std::string select = loadJsonProperty<std::string>(parseJsonObject(request.body), "select");
            if (select == "input") {
                gpio.SetMode(std::stoi(request.matches[1]), GPIOMode::In);
            } else if (select == "output") {
                gpio.SetMode(std::stoi(request.matches[1]), GPIOMode::Out);
            } else if (select == "pwm") {
                gpio.SetMode(std::stoi(request.matches[1]), GPIOMode::PWM);
            } else {
                throw std::runtime_error("Unsupported GPIO mode");
                response.status = 400;
            }
        });
    });

    server.Put("/gpio([0-9]{1,2})/resistor", [&](const httplib::Request &request, httplib::Response &response) {
        handleResource(request, response, [&](const httplib::Request &request, httplib::Response &response) {
            std::string select = loadJsonProperty<std::string>(parseJsonObject(request.body), "select");
            if (select == "pull-up") {
                gpio.SetResistor(std::stoi(request.matches[1]), GPIOResistor::PullUp);
            } else if (select == "pull-down") {
                gpio.SetResistor(std::stoi(request.matches[1]), GPIOResistor::PullDown);
            } else {
                throw std::runtime_error("Unsupported pull up/down resistor type");
                response.status = 400;
            }
        });
    });

    server.Put("/gpio([0-9]{1,2})/pwm", [&](const httplib::Request &request, httplib::Response &response) {
        handleResource(request, response, [&](const httplib::Request &request, httplib::Response &response) {
            nlohmann::json json = parseJsonObject(request.body);
            float periodTime = loadJsonProperty<float>(json, "period-time"), dutyCycle = loadJsonProperty<float>(json, "duty-cycle");
            gpio.SetPWM(std::stoi(request.matches[1]), periodTime, periodTime * dutyCycle);
        });
    });

    std::cout << "Starting GPIO service " << address << ":" << port << std::endl;
    if (!server.listen(address.c_str(), port)) {
        std::cout << "ERROR: Cannot bind service" << std::endl;
    }
    
    return 0;
}
