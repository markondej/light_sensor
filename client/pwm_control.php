<?php

define('SIMULATION', false);
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
		if (!SIMULATION) {
			static::send(strval($level));
		}
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
