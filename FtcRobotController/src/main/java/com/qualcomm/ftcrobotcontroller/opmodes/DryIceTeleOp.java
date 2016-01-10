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

import android.app.backup.RestoreObserver;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

    DcMotor motorClimber;

    TouchSensor touchLeftArm;
    TouchSensor touchRightArm;

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

	float leftArmHoldPower = 0.5f;
	int leftArmHoldPosition = 0;
	float rightArmHoldPower = 0.15f;
	int rightArmHoldPosition =0;
	float leftArmParkPower = 0.2f;
	float rightArmParkPower = 0.1f;
	int armJammedLimit = 30;

    boolean leftArmLocked = false;
    boolean rightArmLocked = false;
    int leftArmLockCounter = 0;
    int rightArmLockCounter = 0;

	// scooper
	Servo scooper;
	float scooperDelta = 0.1f;
	float scooperPosition = 0.1f;
	float scooperMin = 0.01f;
	float scooperMax = 0.99f;
	float scooperParkingPos = scooperMin;

	// elevator
	Servo elevator;
	float elevatorUpPosition = 0.999f;
	float elevatorDownPosition = 0.001f;
	float elevatorStopPosition = 0.52f;

	// dumper
	Servo dumper;
	//float[] dumperPosLUT = { 0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f};
	float[] dumperPosLUT = { 0.25f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f, 0.55f, 0.6f };
	float dumperLoadPosition = dumperPosLUT[0];
	float dumperUnloadPosition = dumperPosLUT[dumperPosLUT.length-1];
	float dumperParkPosition = dumperLoadPosition;

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
            0.6f,  0.7f,  0.8f,  0.9f, 1.0f };

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

        motorBottomLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBottomRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorTopLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorTopRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorClimber = hardwareMap.dcMotor.get("climberM");

		scooper = hardwareMap.servo.get("scooper");
		elevator = hardwareMap.servo.get("elevator");
		dumper = hardwareMap.servo.get("dumper");

        touchLeftArm = hardwareMap.touchSensor.get("touchLeftArm");
        touchRightArm = hardwareMap.touchSensor.get("touchRightArm");

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
		rightArmPowerScale = 0.6f;

        scooper.setPosition(scooperParkingPos);
		elevator.setPosition(elevatorStopPosition);
		dumper.setPosition(dumperParkPosition);

		leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
		rightWheelStartPos = motorBottomRight.getCurrentPosition();
		leftWheelCurrent = leftWheelStartPos;
		rightWheelCurrent = rightWheelStartPos;

        leftArmLocked = false;
        rightArmLocked = false;
        leftArmLockCounter = 0;
        rightArmLockCounter = 0;
	}

    public void start () {
        motorBottomLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBottomRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorTopLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorTopRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }
	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		boolean debug = false;
		if (gamepad1.y)
		{
			debug = true;
		}

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

		if (gamepad1.left_stick_button || gamepad1.a)
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

        // move arms, gamepad1 is the primary
        float throttleArm = -gamepad1.right_stick_y;
        float directionArm = gamepad1.right_stick_x;

        float rightArm = throttleArm - directionArm;
        float leftArm = throttleArm + directionArm;

		// load position
		if (gamepad1.left_bumper || gamepad2.left_bumper) {
			leftArmLockCounter++;
			if (leftArmLockCounter >= 50) {
				if (leftArmLocked) {
					leftArmLocked = false;
				} else {
					leftArmLocked = true;
				}
				leftArmLockCounter = 0;
			}
		}

        if (leftArmLocked) {
			//holdLeftArm(leftArmHoldPosition);
            //ResQUtils.holdMotorByEncoder(motorTopLeft, leftArmLastPos, leftArmHoldPower);
		} else {
            leftArm = Range.clip(leftArm, -1.0f, 1.0f);

            leftArm = ResQUtils.lookUpTableFunc(leftArm, armPowerLUT) * leftArmPowerScale;
            leftArmLastPos = moveLeftArm(leftArm, !gamepad1.b);
        }
		telemetry.addData("left ARM ",
				"locked: " + leftArmLocked + " pwr: " + String.format("%.2f", leftArm)
						+ " pos: " + String.format("%05d", leftArmLastPos));

        // load position
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
			rightArmLockCounter++;
			if (rightArmLockCounter >= 50) {
				if (rightArmLocked) {
					rightArmLocked = false;
				} else {
					rightArmLocked = true;
				}
				rightArmLockCounter = 0;
			}
		}

		if (rightArmLocked) {
            //holdRightArm(rightArmHoldPosition);
            //ResQUtils.holdMotorByEncoder(motorTopRight, rightArmLastPos, rightArmHoldPower);
		} else {
            rightArm = Range.clip(rightArm, -1, 1);

            rightArm = ResQUtils.lookUpTableFunc(rightArm, armPowerLUT) * rightArmPowerScale;
            rightArmLastPos = moveRightArm(rightArm, !gamepad1.b);
        }

		telemetry.addData("right ARM ",
				"locked: " + rightArmLocked + " pwr: " + String.format("%.2f", rightArm)
						+ "pos: " + String.format("%05d", rightArmLastPos));

		// scooper
		if (gamepad1.x) {
            scooper.setPosition(scooperMax);
		} else {
            scooper.setPosition(scooperMin);
		}

		// elevator
		if ((gamepad1.dpad_down && gamepad1.dpad_left) || gamepad2.dpad_down)
		{
			elevator.setPosition(elevatorDownPosition);
		} else if ((gamepad1.dpad_up && gamepad1.dpad_left) || gamepad2.dpad_up)
		{
			elevator.setPosition(elevatorUpPosition);
		}
		else
		{
			elevator.setPosition(elevatorStopPosition);
		}

		telemetry.addData("SCOOPER", "pos: " + String.format("%.2g", scooper.getPosition()));

        // climber
        if ((gamepad1.dpad_down && gamepad1.dpad_right))
        {
            ResQUtils.moveMotorByPower(motorClimber, -0.25f);
        } else if ((gamepad1.dpad_up && gamepad1.dpad_right))
        {
            ResQUtils.moveMotorByPower(motorClimber, 0.25f);
        }
        else
        {
            ResQUtils.moveMotorByPower(motorClimber, (float) Range.clip(gamepad2.right_stick_y, -1.0, 1.0));
        }

		// dumper
		float dumperPos = gamepad1.left_trigger;
		dumper.setPosition(ResQUtils.lookUpTableFunc(dumperPos, dumperPosLUT));
		telemetry.addData("DUMPER", "pos: " + String.format("%.2g", dumper.getPosition()));
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
        motorTopLeft.setPower(0.0f);
		motorTopRight.setPower(0.0f);
        scooper.setPosition(scooperParkingPos);

		motorTopLeft.close();
		motorTopRight.close();
		motorBottomLeft.close();
		motorBottomRight.close();
		scooper.close();
		motorClimber.close();
		elevator.close();
		dumper.close();
		touchLeftArm.close();
		touchRightArm.close();
	}

	boolean isOutOfLeftArmLimit (int pos, float power ){
		//return pos < leftArmUpperLimit && power <0;
//        return (pos < leftArmUpperLimit && power <0)
//				|| ((touchLeftArm.isPressed() || pos > leftArmLowerLimit) && power >0);
        return touchLeftArm.isPressed() && power >0;
	}

	boolean isOutOfRightArmLimit (int pos, float power) {
		//return pos < rightArmUpperLimit && power <0 ;
//        return (pos < rightArmUpperLimit && power <0 )
//				|| ((touchRightArm.isPressed() || pos > rightArmLowerLimit) && power >0);
        return touchRightArm.isPressed() && power > 0;
	}

	int moveLeftArm(float leftArmPower, boolean useLimit)
	{
		// check motor limit
		int leftArmCurrent = motorTopLeft.getCurrentPosition();

        // set motor mode
        if (motorTopLeft.getMode() != DcMotorController.RunMode.RUN_USING_ENCODERS) {
            motorTopLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        // check limit swtich
		if (useLimit && isOutOfLeftArmLimit(leftArmCurrent,leftArmPower)) {
			// do something to prevent jamming
		}
		else if (Math.abs(leftArmLastPower) >= 0.7
				&& Math.abs(leftArmLastPos - leftArmCurrent) < armJammedLimit) {
			motorTopLeft.setPower(leftArmPower * 0.2); // lower power when arm stuck
		} else {
			motorTopLeft.setPower(leftArmPower);
		}

        leftArmLastPower = leftArmPower;
        leftArmLastPos = leftArmCurrent;
		return leftArmCurrent;
	}

	int moveRightArm(float rightArmPower, boolean useLimit)
	{
        int rightArmCurrent = motorTopRight.getCurrentPosition();

        if (motorTopRight.getMode() != DcMotorController.RunMode.RUN_USING_ENCODERS) {
            motorTopRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        if (useLimit && isOutOfRightArmLimit(rightArmCurrent,rightArmPower)){
			// do something to prevent jamming
		}
        else if (Math.abs(rightArmLastPower) >= 0.7
				&& Math.abs(rightArmLastPos - rightArmCurrent) < armJammedLimit) {
			motorTopRight.setPower(rightArmPower * 0.2);
		} else {
			motorTopRight.setPower(rightArmPower);
		}
        rightArmLastPower = rightArmPower;
        rightArmLastPos = rightArmCurrent;
        return rightArmCurrent;
	}

	int moveLeftArmByEncoder(float left, boolean useLimit)
	{
		// check motor limit
		int armCurrent = motorTopLeft.getCurrentPosition();
		int nextPosition = (int)(armCurrent + ResQUtils.lookUpTableFunc(left, armSpeedLUT));

		if (useLimit && touchLeftArm.isPressed()) {
			// do something to prevent jamming
		} else if (!motorTopLeft.isBusy()){
			motorTopLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
			motorTopLeft.setTargetPosition(nextPosition);
			motorTopLeft.setPower(Math.max(leftArmParkPower, Math.abs(left)));
            leftArmLastPower = left;
		}
		leftArmLastPos = armCurrent;
		return armCurrent;
	}

	int moveRightArmByEncoder(float right, boolean useLimit)
	{
		// check motor limit
		int armCurrent = motorTopRight.getCurrentPosition();
		int nextPosition = (int)(armCurrent + ResQUtils.lookUpTableFunc(right, armSpeedLUT));

		if (useLimit && touchRightArm.isPressed()) {
			// do something to prevent jamming
		} else if (!motorTopRight.isBusy()){
			motorTopRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorTopRight.setTargetPosition(nextPosition);
			motorTopRight.setPower(Math.max(rightArmParkPower, Math.abs(right)));
            rightArmLastPower = right;
		}
		rightArmLastPos = armCurrent;
		return armCurrent;
	}

    void holdLeftArm (int pos)
    {
//        motorTopLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        motorTopLeft.setTargetPosition(pos);
//		  motorTopLeft.setPower(leftArmHoldPower);
        ResQUtils.moveMotorByEncoder(motorTopLeft, pos, 10, 300, armPowerLUT);
    }

    void holdRightArm(int pos)
    {
//        motorTopRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        motorTopRight.setTargetPosition(pos);
//		  motorTopRight.setPower(rightArmHoldPower);
        ResQUtils.moveMotorByEncoder(motorTopRight, pos, 10, 400.0, armPowerLUT);
    }

	void moveLeftWheelByEncoder(int pos, float power){
        ResQUtils.holdMotorByEncoder(motorBottomLeft,pos,power);
	}

	void moveRightWheelByEncoder(int pos, float power){
        ResQUtils.holdMotorByEncoder(motorBottomRight, pos, power);
	}

	void moveLeftWheelByPower(float power) {
        ResQUtils.moveMotorByPower(motorBottomLeft,power);
	}
	void moveRightWheelByPower(float power) {
		ResQUtils.moveMotorByPower(motorBottomRight,power);
	}

}
