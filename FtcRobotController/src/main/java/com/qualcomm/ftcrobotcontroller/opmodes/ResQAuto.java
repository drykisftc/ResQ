/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ResQAuto extends ResQTeleOp {

	ColorSensor sensorRGB;

	OpticalDistanceSensor sensorODS;

	float colorSensitivity = 1.25f;

	char targetColor = 'b';

	long startTime =0;

	long timeBudget = 30000; // 30 seconds

    float cruisePower = 0.4f;
    float searchPower = 0.25f;

	/**
	 * Constructor
	 */
	public ResQAuto() {

	}

	int [] stateArray = { 0, // init
			1, // go straight
			2, // turn
			3, // find the beacon
			4, // touch the button
			5, // backup
			6,// find the rampe with correct color
			7,// climb up the ramp
			8  }; // end
	int state = -1;

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {

        super.init();

		sensorRGB = hardwareMap.colorSensor.get("armColor");
		sensorODS = hardwareMap.opticalDistanceSensor.get("armODS");

		state = 0;
	}

	public void start (){

		startTime = System.currentTimeMillis();
        sensorRGB.enableLed(true);
        sensorODS.enableLed(true);
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		if (System.currentTimeMillis() - startTime > timeBudget)
		{
			stop();
		} else {

			switch (state) {
				case 0:
					// go straight
					state = goStraight();
					break;
				case 1:
					// turn
					state = turn();
					break;
				case 2:
					// find the beacon
					searchBeacon();
					state = 3;
					break;
				case 3:
					// touch the button
					touchButton();
					break;
				case 4:
					// backup
					backup();
					break;
				case 5:
					// find the ramp with correct color
					findRamp();
					break;
				case 6:
					// climb up the ramp
					climbRamp();
				default:
					// error

			}
			// go straight

			// turn

			// find the beacon

			// touch the button

			// backup

			// find the rampe with correct color

			// climb up the ramp
		}

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {
        super.stop();
        sensorRGB.enableLed(false);
        sensorODS.enableLed(false);
	}

	int goStraight()
	{
		int stateCode =0;
        char color = getColor(colorSensitivity);

		if ( color == 'b' || color == 'r' ) {
			motorBottomRight.setPower(0);
			motorBottomLeft.setPower(0);
			return 1;
		}
		else
		{
			motorBottomRight.setPower(cruisePower);
			motorBottomLeft.setPower(cruisePower);
		}
		return stateCode;
	}

	int turn(){
		int stateCode =1;

        // turn left until camera see beacon
		
//        if (false)
//        {
//            motorBottomRight.setPower(searchPower);
//            motorBottomLeft.setPower(-cruisePower);
//        }
//        else {
//            stateCode = 2;
//        }

		return stateCode;
	}

	int searchBeacon(){
		int stateCode =0;

        // keep the beacon in center until ODS trigger
        //if ( sensorODS.getLightDetectedRaw() )


		return stateCode;
	}

	int touchButton(){
		int errorCode =0;
		return errorCode;
	}

	int backup(){
		int errorCode =0;
		return errorCode;
	}

	int findRamp(){
		int errorCode =0;
		return errorCode;
	}

	int climbRamp(){
		int errorCode =0;
		return errorCode;
	}

	char getColor (float snrLimit){
		int r = sensorRGB.red();
		int g = sensorRGB.green();
		int b = sensorRGB.blue();

		// find the max
		int m = Math.max(r,g);
		m = Math.max(m,b);

		// if SNR is good
		if (m > (r+g+b)* 0.333 * snrLimit) {
			if (m == g) return 'g';
			if (m == r) return 'r';
			if (m == b) return 'b';
		}
		return 'u';
	}
}

