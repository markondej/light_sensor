<?php

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

define('PWM_SERVICE_ADDRESS', '192.168.43.92');
define('PWM_SERVICE_PORT', 5000);

class PWMController {
    private static function parseResponse($response) {
        if (strlen($response)) {
            $exploded = explode(':', $response);
            if ($exploded[0] != 'OK') {
                if ($exploded[0] == 'ERROR') {
                    throw new Exception("An error occured: " . rtrim($exploded[1], "\r\n"));
                }
            } else {
                return (int)$exploded[1];
            }
        }
        throw new Exception("Communication error occured!");
    }

    private static function send($command) {
        $socketFd = @fsockopen(PWM_SERVICE_ADDRESS, PWM_SERVICE_PORT, $errNo, $errStr, 60);
        if (!$socketFd) {
            throw new Exception("Could not connect to host!");
        }
        if (!fwrite($socketFd, $command . "\r\n")) {
            throw new Exception("Communication error occured!");
        }
        $response = null;
        while (!feof($socketFd)) {
            $read = fread($socketFd, 1024);
            if ($read === false) {
                throw new Exception("Communication error occured!");
            }
            $response .= $read;
        }
        fclose($socketFd);
        return static::parseResponse($response);
    }

    public static function setLevel($level) {
        static::send(strval($level));
    }
}

try {
    if (!is_null($_REQUEST['level']) && in_array(intval($_REQUEST['level']), [0, 1])) {
        PWMController::setLevel($_REQUEST['level']);
        echo "OK";
    } else {
        echo "Please specify correct 'level' value!";
    }
} catch (Exception $exception) {
    echo $exception->getMessage();
}
