# rpi_gpio_service
Raspberry Pi GPIO RESTful API written in C++

The aim of this project is to bring up simple RESTful API for Raspberry Pi device which would allow controlling GPIO ports remotely by managing proper REST resources. This services emulates PWM via built-in PWM clock and DMA conttroler enabling multi-channel PWM support.

## RESTful API
By default service binds HTTP server to port 8080. Input and output structures are JSON-formatted. Available resources are listed below:

```
Resource            |   Methods    |   Request data example              |   Description
------------------------------------------------------------------------------------------------------------------------------------------------
/gpio[0-27]         |   PUT, GET   |   {"active":true}                   |   Reads/sets state on given GPIO
/gpio[0-27]/mode    |   PUT        |   {"select":"output"}               |   Selects current mode for given GPIO (use: "input", "output", "pwm")
/gpio[0-27]/pwm     |   PUT        |   {"duty-cyle":0.4,"period-time":2} |   Sets PWM parameters (period time in miliseconds)
/gpio[0-27]/resistor|   PUT        |   {"select":"pull-down"}            |   Pull up/down resistor configuration (use: "pull-up", "pull-down")
```

## Features
* No dependencies except of Broadcom libraries required, works on "light" distros  
* Works directly on peripherals, no additional drivers needed
* Multi-channel PWM support, PWM on any available GPIO pin
* Full GPIO input and output control
* Pull-Up and Pull-Down resistors control
* Works on any Raspberry Pi board

## Instalation
To build and run service on Raspberry Pi clone this repository and use "make" command, eg.:
```
git clone https://github.com/markondej/rpi_gpio_service
cd rpi_gpio_service
make
sudo ./gpio_service -p 80
```
