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
public class DryIceTeleOp extends OpMode {

	DcMotor motorBottomRight;
	DcMotor motorBottomLeft;
	DcMotor motorTopRight;
	DcMotor motorTopLeft;

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
	int armMaxDelta = 3;

	float leftArmHoldPower = 0.99f;
	int leftArmHoldPosition = 0;
	float rigthtArmHoldPower = 0.99f;
	int rightArmHoldPosition =0;
	float leftArmParkPower = 0.05f;
	float rightArmParkPower = 0.1f;
	int armJammedLimit = 30;

	// scooper
	Servo scooper;
	float scooperDelta = 0.1f;
	float scooperPosition = 0.1f;
	float scooperParkingPos = 0.9f;
	float scooperMin = 0.1f;
	float scooperMax = 0.9f;

	// elevator
	Servo elevator;
	float elevatorUpPosition = 0.999f;
	float elevatorDownPosition = 0.001f;
	float elevatorStopPosition = 0.52f;

	// dumper
	Servo dumper;
	float[] dumperPosLUT = { 0.88f, 0.85f, 0.80f, 0.75f, 0.70f, 0.65f, 0.60f,
			0.55f, 0.50f, 0.45f, 0.40f, 0.25f, 0.2f, 0.15f, 0.1f, 0.05f, 0.0f};
	float dumperLoadPosition = dumperPosLUT[0];
	float dumperUnloadPosition = dumperPosLUT[dumperPosLUT.length-1];
	float dumperParkPostion= dumperLoadPosition;

	float[] wheelPowerLUT = {0.0f, 0.05f, 0.15f, 0.18f, 0.20f,
			0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
			0.38f, 0.42f, 0.46f, 0.50f, 0.54f, 0.58f, 0.62f, 0.66f,
			0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f, 1.00f };

	float [] wheelSpeedLUT = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f,
			8.0f, 9.0f, 10.0f, 11.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f,
	        80.0f, 90.0f, 100.0f};

	float[] armPowerLUT = { 0.0f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f,
			0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f,
			0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f, 0.23f, 0.24f,
			0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f,
			0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.40f,
			0.41f, 0.42f, 0.43f, 0.44f, 0.45f, 0.46f, 0.47f, 0.48f,
			0.49f, 0.50f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f, 0.56f,
			0.60f, 0.65f, 0.70f, 0.75f, 0.80f, 0.85f, 0.90f, 0.95f, 1.0f };

	float [] armSpeedLUT = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f,
			8.0f, 9.0f, 10.0f, 11.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f,
			80.0f, 90.0f, 100.0f, 110.0f, 120.0f, 140.0f, 160.0f, 180.0f, 200.0f};

	int leftWheelStartPos =0;
	int rightWheelStartPos =0;
	int leftWheelCurrent = 0;
	int rightWheelCurrent = 0;
	float leftWheelTractionControlPower = 1.0f;
	float rightWheelTractionControlPower = 1.0f;
	/**
	 * Constructor
	 */
	public DryIceTeleOp() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {

		motorBottomRight = hardwareMap.dcMotor.get("bottomRightM");
		motorBottomRight.setDirection(DcMotor.Direction.REVERSE);
		motorBottomLeft = hardwareMap.dcMotor.get("bottomLeftM");
		//motorBottomLeft.setDirection(DcMotor.Direction.REVERSE);

		motorTopRight = hardwareMap.dcMotor.get("topRightM");
		//motorTopRight.setDirection(DcMotor.Direction.REVERSE);
		motorTopLeft = hardwareMap.dcMotor.get("topLeftM");
		motorTopLeft.setDirection(DcMotor.Direction.REVERSE);

		scooper = hardwareMap.servo.get("scooper");
		elevator = hardwareMap.servo.get("elevator");
		dumper = hardwareMap.servo.get("dumper");

		// assign the starting position of the wrist and claw
		leftArmLastPos = motorTopLeft.getCurrentPosition();
        leftArmLastReqPos = leftArmLastPos;
		leftArmUpperLimit = leftArmLastPos +100;
		leftArmLoadPosition = leftArmLastPos + 3300;
		leftArmUnloadPosition = leftArmLoadPosition;
		leftArmLowerLimit = leftArmLastPos + 5000;
		leftArmLastPower = 0.0f;
		leftArmPowerScale = 0.6f;

		rightArmLastPos = motorTopRight.getCurrentPosition();
        rightArmLastReqPos = rightArmLastPos;
		rightArmUpperLimit = rightArmLastPos +100;
		rightArmLoadPosition = rightArmLastPos + 3300;
		rightArmUnloadPosition = rightArmLoadPosition -500;
		rightArmLowerLimit = rightArmLastPos + 5000;
		rightArmLastPower = 0.0f;
		rightArmPowerScale = 0.3f;

        scooper.setPosition(scooperParkingPos);
		elevator.setPosition(elevatorStopPosition);
		dumper.setPosition(dumperParkPostion);

		leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
		rightWheelStartPos = motorBottomRight.getCurrentPosition();
		leftWheelCurrent = leftWheelStartPos;
		rightWheelCurrent = rightWheelStartPos;
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
		float throttle  = -gamepad1.left_stick_y;
		float direction = gamepad1.left_stick_x;

		// driving power
		float right = throttle - direction;
		float left = throttle + direction;

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left  = Range.clip(left, -1, 1);

		leftWheelCurrent = motorBottomLeft.getCurrentPosition();
		rightWheelCurrent = motorBottomRight.getCurrentPosition();

		if ((gamepad1.left_stick_button || gamepad1.a)
				&& (Math.abs(left)>=0.02 || Math.abs(right) >=0.02) )
		{
			// traction control mode
			float leftSpeed = ResQUtils.lookUpTableFunc(left, wheelSpeedLUT);
			float rightSpeed = ResQUtils.lookUpTableFunc(right,wheelSpeedLUT);
			moveLeftWheelByEncoder((int)(leftWheelCurrent + leftSpeed), leftWheelTractionControlPower);
			moveRightWheelByEncoder((int)(rightWheelCurrent + rightSpeed), rightWheelTractionControlPower);

			// logging
			telemetry.addData("left WHEEL ", "speed: " + String.format("%.2f", leftSpeed)
					+ " distance: " + String.format("%05d", leftWheelCurrent - leftWheelStartPos));
			telemetry.addData("right WHEEL", "speed: " + String.format("%.2f", rightSpeed)
					+ " distance: " + String.format("%05d", rightWheelCurrent - rightWheelStartPos));
		}
		else {
			// scale the joystick value to make it easier to control
			// the robot more precisely at slower speeds.
			right = ResQUtils.lookUpTableFunc(right, wheelPowerLUT);
			left = ResQUtils.lookUpTableFunc(left, wheelPowerLUT);

			// move wheels
            moveLeftWheelByPower(left);
			moveRightWheelByPower(right);
			// logging
			telemetry.addData("left WHEEL ", "pwr: " + String.format("%.2f", left)
					+ " distance: " + String.format("%05d", leftWheelCurrent - leftWheelStartPos));
			telemetry.addData("right WHEEL", "pwr: " + String.format("%.2f", right)
					+ " distance: " + String.format("%05d", rightWheelCurrent - rightWheelStartPos));
		}

        // move arms
        float throttleArm = -gamepad1.right_stick_y;
        float directionArm = gamepad1.right_stick_x;
        float rightArm = throttleArm - directionArm;
        float leftArm = throttleArm + directionArm;

		// load position
		if (gamepad1.left_bumper) {
			if (leftArmHoldPosition == 0){
				leftArmHoldPosition = leftArmLastPos;
			}
			holdLeftArm(leftArmHoldPosition);
		} else {
			leftArmHoldPosition = 0;
			leftArm = Range.clip(leftArm, -1, 1);

			if (gamepad1.right_stick_button) {
				leftArm = ResQUtils.lookUpTableFunc(leftArm, armPowerLUT) * leftArmPowerScale;
				leftArmLastPos = moveLeftArm(leftArm, !gamepad1.b);
			} else  {
				leftArmLastPos = moveLeftArmByEncoder(leftArm, !gamepad1.b);
			}

			telemetry.addData("left ARM ",
					"pwr: " + String.format("%.2f", leftArm)
							+ " pos: " + String.format("%05d", leftArmLastPos));
		}

		if (gamepad1.right_bumper) {
			if (rightArmHoldPosition ==0) {
				rightArmHoldPosition = rightArmLastPos;
			}
			holdRightArm(rightArmHoldPosition);
		} else {
			rightArmHoldPosition = 0;
			rightArm = Range.clip(rightArm, -1, 1);

			if (gamepad1.right_stick_button) {
				rightArm = ResQUtils.lookUpTableFunc(rightArm, armPowerLUT) * rightArmPowerScale;
				rightArmLastPos = moveRightArm(rightArm, !gamepad1.b);
			} else {
				rightArmLastPos = moveRightArmByEncoder(rightArm, !gamepad1.b);
			}
			telemetry.addData("right ARM ",
					"pwr: " + String.format("%.2f", rightArm)
							+ "pos: " + String.format("%05d", rightArmLastPos));
		}

		// scooper
		if (gamepad1.x) {
            scooper.setPosition(scooperMin);
		} else {
            scooper.setPosition(scooperMax);
		}

		// elevator
		if (gamepad1.dpad_down || gamepad2.dpad_down)
		{
			elevator.setPosition(elevatorDownPosition);
		} else if (gamepad1.dpad_up || gamepad2.dpad_up)
		{
			elevator.setPosition(elevatorUpPosition);
		}
		else
		{
			elevator.setPosition(elevatorStopPosition);
		}
		telemetry.addData("SCOOPER", "pos: " + String.format("%.2g", scooper.getPosition()));

		// dumper
		if (gamepad1.y) {
			dumper.setPosition(dumperUnloadPosition);
		} else if (gamepad1.right_trigger >= 0.01) {
			dumper.setPosition(ResQUtils.lookUpTableFunc(gamepad1.right_trigger, dumperPosLUT));
		}
		else {
			dumper.setPosition(dumperLoadPosition);
		}
		telemetry.addData("DUMPER", "pos: " + String.format("%.2g", scooper.getPosition()));

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
        moveLeftArm(0.0f,true);
        moveRightArm(0.0f, true);
        scooper.setPosition(scooperParkingPos);
	}

	boolean isOutOfLeftArmLimit (int pos, float power ){
		return (pos < leftArmUpperLimit && power <0) || (pos > leftArmLowerLimit && power >0);
	}

	boolean isOutOfRightArmLimit (int pos, float power) {
		return (pos < rightArmUpperLimit && power <0 ) || (pos > rightArmLowerLimit && power >0);
	}

	int moveLeftArm(float leftArmPower, boolean useLimit)
	{
		// check motor limit
		int leftArmCurrent = motorTopLeft.getCurrentPosition();
        motorTopLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		if (useLimit && isOutOfLeftArmLimit(leftArmCurrent, leftArmPower)) {
			// do something to prevent jamming
		}
		else if (Math.abs(leftArmLastPower) >= 0.5
				&& Math.abs(leftArmLastPos - leftArmCurrent) < armJammedLimit) {
			motorTopLeft.setPower(leftArmLastPower * 0.2); // lower power when arm stuck
		} else {
			motorTopLeft.setPower(leftArmPower);
		}
        leftArmLastPower = leftArmPower;
		return leftArmCurrent;
	}

	int moveRightArm(float rightArmPower, boolean useLimit)
	{
        int rightArmCurrent = motorTopRight.getCurrentPosition();
        motorTopRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        if (useLimit && isOutOfRightArmLimit(rightArmCurrent, rightArmPower)){
			// do something to prevent jamming
		}
        else if (Math.abs(rightArmLastPower) >= 0.5
				&& Math.abs(rightArmLastPos - rightArmCurrent) < armJammedLimit) {
			motorTopRight.setPower(rightArmLastPower * 0.2);
		} else {
			motorTopRight.setPower(rightArmPower);
		}

        rightArmLastPower = rightArmPower;
        return rightArmCurrent;
	}

	int moveLeftArmByEncoder(float left, boolean useLimit)
	{
		// check motor limit
		int armCurrent = motorTopLeft.getCurrentPosition();
		int nextPosition = (int)(armCurrent + ResQUtils.lookUpTableFunc(left, armSpeedLUT));

		if (useLimit && isOutOfLeftArmLimit(nextPosition, left)) {
			// do something to prevent jamming
		} else {
			motorTopLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
			motorTopLeft.setTargetPosition(nextPosition);
			motorTopLeft.setPower(Math.max(leftArmParkPower,Math.abs(left)));
		}
		leftArmLastPower = left;
		leftArmLastPos = armCurrent;
		return armCurrent;
	}

	int moveRightArmByEncoder(float right, boolean useLimit)
	{
		// check motor limit
		int armCurrent = motorTopRight.getCurrentPosition();
		int nextPosition = (int)(armCurrent + ResQUtils.lookUpTableFunc(right, armSpeedLUT));

		if (useLimit && isOutOfRightArmLimit(nextPosition, right)) {
			// do something to prevent jamming
		} else {
			motorTopRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
			motorTopRight.setTargetPosition(nextPosition);
			motorTopRight.setPower(Math.max(rightArmParkPower, Math.abs(right)));
		}
		rightArmLastPower = right;
		rightArmLastPos = armCurrent;
		return armCurrent;
	}

    void holdLeftArm (int pos)
    {
        motorTopLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorTopLeft.setTargetPosition(pos);
		motorTopLeft.setPower(leftArmHoldPower);
    }

    void holdRightArm(int pos)
    {
        motorTopRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorTopRight.setTargetPosition(pos);
		motorTopRight.setPower(rigthtArmHoldPower);
    }

	void moveLeftWheelByEncoder(int pos, float power){
		motorBottomLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		motorBottomLeft.setTargetPosition(pos);
		motorBottomLeft.setPower(power);
	}

	void moveRightWheelByEncoder(int pos, float power){
		motorBottomRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		motorBottomRight.setTargetPosition(pos);
		motorBottomRight.setPower(power);
	}

	void moveLeftWheelByPower(float power) {
		motorBottomLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		motorBottomLeft.setPower(power);
	}
	void moveRightWheelByPower(float power) {
		motorBottomRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		motorBottomRight.setPower(power);
	}

//    int moveLeftArmDeltaPosition(float delta, int maxDelta)
//    {
//        int armCurrent = motorTopLeft.getCurrentPosition();
//        int nextPosition = scaleInputArmPositionDelta(delta, maxDelta) + armCurrent;
//        if (nextPosition > leftArmUpperLimit && nextPosition < leftArmLowerLimit ) {
//            motorTopLeft.setTargetPosition(nextPosition);
//            leftArmLastReqPos = nextPosition;
//        }
//        else {
//            moveLeftArm(delta);
//        }
//        return armCurrent;
//    }
//
//    int moveRightArmDeltaPosition(float delta, int maxDelta)
//    {
//        int rightArmCurrent = motorTopRight.getCurrentPosition();
//        int nextPosition = scaleInputArmPositionDelta(delta, maxDelta) + rightArmCurrent;
//        if (nextPosition > rightArmUpperLimit && nextPosition < rightArmLowerLimit ) {
//            motorTopRight.setTargetPosition(nextPosition);
//            rightArmLastReqPos = nextPosition;
//        }
//        else {
//            moveRightArm(delta);
//        }
//        return rightArmCurrent;
//    }
}
