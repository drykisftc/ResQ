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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ResQTeleOp extends OpMode {

    int armMaxDelta = 3;

	// position of the left arm
	int leftArmLastPos;
    int leftArmLastReqPos;
	float leftArmLastPower;
	int leftArmUpperLimit;
	int leftArmLowerLimit;
	int leftArmLoadPosition;
	int leftArmUnloadPosition;
	float leftArmPowerScale;

	// position of the right arm
	int rightArmLastPos;
    int rightArmLastReqPos;
	float rightArmLastPower;
	int rightArmUpperLimit;
	int rightArmLowerLimit;
	int rightArmLoadPosition;
	int rightArmUnloadPosition;
	float rightArmPowerScale;

	// amount to change the arm servo position.
	float scooperDelta = 0.1f;
	float scooperPosition = 0.1f;
    float scooperParkingPos = 0.6f;
    float scooperMin = 0.0f;
    float scooperMax = 1.0f;

	DcMotor motorBottomRight;
	DcMotor motorBottomLeft;
	DcMotor motorTopRight;
	DcMotor motorTopLeft;

	Servo scooper;

	/**
	 * Constructor
	 */
	public ResQTeleOp() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {

		motorBottomRight = hardwareMap.dcMotor.get("bottomRightM");
		motorBottomLeft = hardwareMap.dcMotor.get("bottomLeftM");
		motorBottomLeft.setDirection(DcMotor.Direction.REVERSE);

		motorTopRight = hardwareMap.dcMotor.get("topRightM");
		motorTopLeft = hardwareMap.dcMotor.get("topLeftM");
		motorTopRight.setDirection(DcMotor.Direction.REVERSE);

		scooper = hardwareMap.servo.get("scooper");

		// assign the starting position of the wrist and claw
		leftArmLastPos = motorTopLeft.getCurrentPosition();
        leftArmLastReqPos = leftArmLastPos;
		leftArmUpperLimit = leftArmLastPos +100;
		leftArmLoadPosition = leftArmLastPos + 3300;
		leftArmUnloadPosition = leftArmLoadPosition;
		leftArmLowerLimit = leftArmLastPos + 5000;
		leftArmLastPower = 0.0f;
		leftArmPowerScale = 0.5f;

		rightArmLastPos = motorTopRight.getCurrentPosition();
        rightArmLastReqPos = rightArmLastPos;
		rightArmUpperLimit = rightArmLastPos +100;
		rightArmLoadPosition = rightArmLastPos + 3300;
		rightArmUnloadPosition = rightArmLoadPosition -500;
		rightArmLowerLimit = rightArmLastPos + 5000;
		rightArmLastPower = 0.0f;
		rightArmPowerScale = 0.3f;

	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 *
		 * Left joystick controls the wheel motors. Right joystick controls the arms
		 */

		// throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
		// 1 is full down
		// direction: left_stick_x ranges from -1 to 1, where -1 is full left
		// and 1 is full right
		float throttle = -gamepad1.left_stick_y;
		float direction = gamepad1.left_stick_x;
		float right = throttle - direction;
		float left = throttle + direction;


		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = scaleInput(right);
		left =  scaleInput(left);

        // move wheels
        motorBottomRight.setPower(right);
        motorBottomLeft.setPower(left);

        // move arms
        float throttleArm = -gamepad1.right_stick_y;
        float directionArm = gamepad1.right_stick_x;
        float rightArm = throttleArm - directionArm;
        float leftArm = throttleArm + directionArm;

		// load position
		if (gamepad1.dpad_down)
		{
            motorTopLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorTopRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
			motorTopLeft.setTargetPosition(leftArmLoadPosition);
			motorTopRight.setTargetPosition(rightArmLoadPosition);
			telemetry.addData("load debris",
					"left:" + String.format("%05d", leftArmLoadPosition)
							+ " right:"+String.format("%05d", rightArmLoadPosition));
		} else if (gamepad1.dpad_right) // unload position
		{
            motorTopLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorTopRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
			motorTopLeft.setTargetPosition(leftArmUnloadPosition);
			motorTopRight.setTargetPosition(rightArmUnloadPosition);
			telemetry.addData("Unload debris",
					"left:" + String.format("%05d", leftArmLoadPosition)
							+ " right:" + String.format("%05d", rightArmLoadPosition));
		}
		else {

			if (gamepad1.left_bumper) {
				holdLeftArm();
			} else {
				leftArm = Range.clip(leftArm, -1, 1);
				//int leftArmCurrent = moveLeftArmDeltaPosition(leftArm, armMaxDelta);
				leftArm = scaleInputArm(leftArm) * leftArmPowerScale;
				leftArmLastPos = moveLeftArm(leftArm);
                telemetry.addData("left ARM ",
                        "pwr: " + String.format("%.2f", leftArm)
                                + " pos: " + String.format("%05d", leftArmLastPos));
			}

			if (gamepad1.right_bumper) {
				holdRightArm();
			} else {
				rightArm = Range.clip(rightArm, -1, 1);
				//int rightArmCurrent = moveRightArmDeltaPosition(rightArm, armMaxDelta);
				rightArm = (float) scaleInputArm(rightArm) * rightArmPowerScale;
				rightArmLastPos = moveRightArm(rightArm);
                telemetry.addData("right ARM ",
                        "pwr: " + String.format("%.2f", rightArm)
                                + "pos: " + String.format("%05d", rightArmLastPos));
			}
		}

		// update the position of the arm.
		if (gamepad1.x) {
            scooper.setDirection(Servo.Direction.REVERSE);
            scooper.setPosition(0.6);
		} else {
		}

        // logging
        telemetry.addData("left WHEEL ", "pwr: " + String.format("%.2f", left));
        telemetry.addData("right WHEEL", "pwr: " +String.format("%.2f", right));
        telemetry.addData("scooper", "pos: " +String.format("%.2g", scooper.getPosition()));
    }

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {
        motorBottomRight.setPower(0.0f);
        motorBottomLeft.setPower(0.0f);
        moveLeftArm(0.0f);
        moveRightArm(0.0f);
	}

	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	float scaleInputArm(float dVal)  {
		float[] scaleArray = { 0.0f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f,
				0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f,
				0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f, 0.23f, 0.24f,
				0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f,
				0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.40f,
				0.41f, 0.42f, 0.43f, 0.44f, 0.45f, 0.46f, 0.47f, 0.48f,
				0.49f, 0.50f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f, 0.56f,
				0.60f, 0.65f, 0.70f, 0.75f, 0.80f, 0.85f, 0.90f, 0.95f, 1.0f };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 64.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 64) {
			index = 64;
		}

		// get value from the array.
		float dScale = 0.0f;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	float scaleInput(float dVal)  {
		float[] scaleArray = { 0.0f, 0.05f, 0.09f, 0.10f, 0.12f, 0.15f, 0.18f, 0.20f,
				0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
				0.38f, 0.42f, 0.46f, 0.50f, 0.54f, 0.58f, 0.62f, 0.66f,
				0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f, 1.00f };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 32.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 32) {
			index = 32;
		}

		// get value from the array.
		float dScale = 0.0f;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

    int scaleInputArmPositionDelta(float dVal, int maxDelta)  {
        float[] scaleArray = { 0.0f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f,
                0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f,
                0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f, 0.23f, 0.24f,
                0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f,
                0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.40f,
                0.42f, 0.44f, 0.46f, 0.48f, 0.5f, 0.52f, 0.54f, 0.56f,
                0.58f, 0.60f, 0.62f, 0.64f, 0.66f, 0.68f, 0.70f, 0.72f,
                0.75f, 0.80f, 0.83f, 0.86f, 0.89f, 0.92f, 0.95f, 0.98f, 1.0f };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 64.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 64) {
            index = 64;
        }

        // get value from the array.
        float dScale = 0.0f;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return (int)dScale*maxDelta;
    }

	int moveLeftArm(float leftArmPower)
	{
		// check motor limit
        motorTopLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		int leftArmCurrent = motorTopLeft.getCurrentPosition();
		if ( leftArmCurrent< leftArmUpperLimit && leftArmPower <0) {
			// do something to prevent jamming
		}
		else if  (leftArmCurrent > leftArmLowerLimit && leftArmPower >0) {
			// do something to prevent jamming
		}
		else {
			// check whether motors go stuck
			if (Math.abs(leftArmLastPower) >= 0.5 && Math.abs(leftArmLastPos - leftArmCurrent) < 30) {
				motorTopLeft.setPower(leftArmLastPower*0.5); // lower power when arm stuck
			} else {
				motorTopLeft.setPower(leftArmPower);
			}
		}
        leftArmLastPower = leftArmPower;
		return leftArmCurrent;
	}

	int moveRightArm(float rightArmPower)
	{
        int rightArmCurrent = motorTopRight.getCurrentPosition();
        motorTopRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        if (rightArmCurrent < rightArmUpperLimit && rightArmPower <0 ) {
            // do something to prevent jamming
        }
        else if (rightArmCurrent > rightArmLowerLimit && rightArmPower >0)
        {
            // do something to prevent jamming
        }
        else {
            if (Math.abs(rightArmLastPower) >= 0.5 && Math.abs(rightArmLastPos - rightArmCurrent) < 30) {
                motorTopRight.setPower(rightArmLastPower*0.5);
            } else {
                motorTopRight.setPower(rightArmPower);
            }
        }
        rightArmLastPower = rightArmPower;
        return rightArmCurrent;
	}

    void holdLeftArm ()
    {
        motorTopLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorTopLeft.setTargetPosition(leftArmLastPos);
    }

    void holdRightArm()
    {
        motorTopRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorTopRight.setTargetPosition(rightArmLastPos);
    }

    int moveLeftArmDeltaPosition(float delta, int maxDelta)
    {
        int armCurrent = motorTopLeft.getCurrentPosition();
        int nextPosition = scaleInputArmPositionDelta(delta, maxDelta) + armCurrent;
        if (nextPosition > leftArmUpperLimit && nextPosition < leftArmLowerLimit ) {
            motorTopLeft.setTargetPosition(nextPosition);
            leftArmLastReqPos = nextPosition;
        }
        else {
            moveLeftArm(delta);
        }
        return armCurrent;
    }

    int moveRightArmDeltaPosition(float delta, int maxDelta)
    {
        int rightArmCurrent = motorTopRight.getCurrentPosition();
        int nextPosition = scaleInputArmPositionDelta(delta, maxDelta) + rightArmCurrent;
        if (nextPosition > rightArmUpperLimit && nextPosition < rightArmLowerLimit ) {
            motorTopRight.setTargetPosition(nextPosition);
            rightArmLastReqPos = nextPosition;
        }
        else {
            moveRightArm(delta);
        }
        return rightArmCurrent;
    }
}
