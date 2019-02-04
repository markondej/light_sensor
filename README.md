# rpi_cpp_gpio_controll
Raspberry Pi GPIO control library for C++

Features:
* works directly on peripherals, no additional drivers needed
* multiple PWM support (multiple servos control), PWM on any available GPIO pin
* supports input and output control
* supports Pull-Up and Pull-Down resistors control

Usage:
```
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
```