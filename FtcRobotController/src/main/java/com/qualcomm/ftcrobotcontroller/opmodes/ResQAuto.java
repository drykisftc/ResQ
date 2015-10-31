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

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ResQAuto extends ResQTeleOp {

		ColorSensor sensorRGB;

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

		state = 0;
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		switch (state)
		{
			case 0:
				// go straight
				goStraight();
				state = 1;
                break;
			case 1:
				// turn
				turn();
                state =2;
				break;
			case 2:
				// find the beacon
                searchBeacon();
				state =3;
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

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

	int goStraight()
	{
		int errorCode =0;
		if ( true) {
			motorBottomRight.setPower(0);
			motorBottomLeft.setPower(0);
		}
		else
		{
			motorBottomRight.setPower(1.0);
			motorBottomLeft.setPower(1.0);
		}
		return errorCode;
	}

	int turn(){
		int errorCode =0;
		return errorCode;
	}

	int searchBeacon(){
		int errorCode =0;
		return errorCode;
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
}

