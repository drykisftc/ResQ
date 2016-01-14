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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Queue;

/**
 * Auto Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class DryIceAuto extends DryIceTeleOp {

    ColorSensor sensorRGB;

    OpticalDistanceSensor sensorODSLeft;
    OpticalDistanceSensor sensorODSRight;

    GyroSensor sensorGyro;

    //ResQCamera camera;

    boolean enableLED = true;

    float colorSensitivity = 1.25f;

    char teamColor = 'b';

    long startTime = 0;

    long timeBudget = 30000; // 30 seconds
    long lastStateTimeStamp = 0;

    float cruisePower = 0.99f;
    float scoutPower = 0.75f;
    float searchPower = 0.5f;
    float turnPower = 0.3f;

    int collisionDistThreshold = 15;
    int minColorBrightness = 8;
    TurnData prevTurnData;
    double prevTurnPower = 0.0;

    int refGyro = 0;
    int prevGyro = 0;
    int currentGyro = 0;
    int targetAngle = 0;
    int targetAngleTolerance = 2;
    float[] angle2PowerLUT = {0.00f, 0.01f, 0.015f, 0.02f, 0.025f, 0.03f, 0.035f, 0.04f, 0.045f,0.05f,
                              0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f,
                              0.07f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f,
                              0.08f, 0.08f, 0.08f, 0.08f, 0.08f, 0.08f, 0.08f, 0.08f, 0.08f, 0.08f,
                              0.09f, 0.09f, 0.09f, 0.09f, 0.09f, 0.09f, 0.09f, 0.09f, 0.09f, 0.09f,
                              0.1f,  0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f};

    float[] angle2DistanceLUT = {0.00f, 2.0f, 5.1f, 7.5f, 10.15f, 15.2f, 20.25f, 30.3f, 40.35f, 50.4f};
    float lastSkew =0;
    float skewPowerScale = 1.0f;
    float skewPowerGain = 1.02f;

    GyroData gyroData;
    int refXRotation = 0;
    int refYRotation = 0;
    int refZRotation = 0;

    RGB rgb= new RGB(0,0,0);

    Queue<TurnData> turnDataFIFO;

    // 3759 per block
    int StarLineToCenterLineDistance = 1500;
    int CenterlineToBeaconLineDistance = 6400;
    int BeaconLineToBeaconDistance = 4500;
    int RampLineToBeaconLineDistance = 3500;
    int turnAngle = 45;

    /**
     * Constructor
     */
    public DryIceAuto() {

    }

    int[] stateArray = {0, // init
            1, // go straight
            2, // turn
            3, // find the beacon
            4, // touch the button
            5, // backup
            6,// find the rampe with correct color
            7,// climb up the rampz
            8}; // end
    int stateDryIce = -1;

    /*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
    @Override
    public void init() {

        super.init();

        enableLED = true;

        sensorODSLeft = hardwareMap.opticalDistanceSensor.get("leftODS");
        sensorODSLeft.enableLed(enableLED);
        sensorODSRight = hardwareMap.opticalDistanceSensor.get("rightODS");
        sensorODSRight.enableLed(enableLED);

        sensorRGB = hardwareMap.colorSensor.get("armColor");
        sensorRGB.enableLed(enableLED);

        sensorGyro = hardwareMap.gyroSensor.get("headGYRO");
        sensorGyro.calibrate();

        gyroData = new GyroData(0,0,0,0);

        //camera = new ResQCamera();
        motorBottomLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBottomRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorTopLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorTopRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        // wait 1 second for gyro calibration
        try {
            Thread.sleep(1500);                 //1000 milliseconds is one second.
        } catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        stateDryIce = 0;
        telemetry.addData("TEAM", "Team color:" + teamColor);
        telemetry.addData("STATE", "stateDryIce: Init done");
        //currentGyro = getGyroHeading();
        //upateRotationData();
        ResQUtils.getGyroData(sensorGyro, gyroData);
        currentGyro = gyroData.heading;
        refGyro = currentGyro;
        prevGyro = currentGyro;
        targetAngle = currentGyro;
        telemetry.addData("GYRO", "GYRO: " + String.format("%03d", targetAngle)
                + " (" + String.format("%03d", currentGyro)
                + " ," + String.format("%03d", gyroData.xRotation)
                + " ," + String.format("%03d", gyroData.yRotation)
                + " ," + String.format("%03d", gyroData.zRotation)+")");
        telemetry.addData("READY"," wait 2 more seconds and go!!!!!");

    }

    public void start() {

        motorBottomLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBottomRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorTopLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorTopRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        //currentGyro = getGyroHeading();
        //upateRotationData();
        ResQUtils.getGyroData(sensorGyro, gyroData);
        currentGyro = gyroData.heading;
        refGyro = currentGyro;
        prevGyro = currentGyro;
        refXRotation = gyroData.xRotation;
        refYRotation = gyroData.yRotation;
        refZRotation = gyroData.zRotation;

        // set the next stateDryIce
        targetAngle = currentGyro;
        telemetry.addData("GYRO", "GYRO: " + String.format("%03d", targetAngle)
                + " (" + String.format("%03d", currentGyro)
                + " ," + String.format("%03d", gyroData.xRotation)
                + " ," + String.format("%03d", gyroData.yRotation)
                + " ," + String.format("%03d", gyroData.zRotation) + ")");

        startTime = System.currentTimeMillis();
        lastStateTimeStamp = startTime;
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

        telemetry.addData("STATE", stateDryIce);

        if (System.currentTimeMillis() - startTime > timeBudget) {
            stop();
        } else {
            prevGyro = currentGyro;

            leftWheelCurrent = motorBottomLeft.getCurrentPosition();
            rightWheelCurrent = motorBottomLeft.getCurrentPosition();

            //currentGyro = getGyroHeading();
            //upateRotationData();
            ResQUtils.getGyroData(sensorGyro, gyroData);
            currentGyro = gyroData.heading;
            telemetry.addData("GYRO", "GYRO: " + String.format("%03d", targetAngle)
                    + " (" + String.format("%03d", currentGyro)
                    + " ," + String.format("%03d", gyroData.xRotation)
                    + " ," + String.format("%03d", gyroData.yRotation)
                    + " ," + String.format("%03d", gyroData.zRotation)+")");
            telemetry.addData("DISTANCE", "Left:" + String.format("%05d", leftWheelCurrent-leftWheelStartPos)
                    + ". Right:" + String.format("%05d", rightWheelCurrent-rightWheelStartPos));

            switch (stateDryIce) {
                case 0:
                    // go straight
                    stateDryIce = goStraightToCenterLine(0, 1, cruisePower, 8000); // should finish in 8 sec
                    break;
                case 1:
                    // turn
                    stateDryIce = turn(1, 2, 0.0f, currentGyro, targetAngle);
                    break;
                case 2:
                    // go straight
                    turnAngle = 2;
                    stateDryIce = goStraightFromCenterlineToBeaconLine(2, 3, scoutPower,20000); // should finish in 20 sec
                    break;
                case 3:
                    // turn
                    stateDryIce = turn(3, 4, 0.0f, currentGyro, targetAngle);
                    break;
                case 4:
                    stateDryIce = goStraightFromBeaconlineToBeacon(4,5, searchPower, 25000);
                    // find the beacon
                    //stateDryIce = searchBeacon(searchPower, 4, 5);
                    break;
                case 5:
                    // touch the button
                    stateDryIce = touchButton(5,6);
                    break;
                case 6:
                    // backup
                    stateDryIce = backup(6,7);
                    break;
                case 7:
                    // find the ramp with correct color
                    stateDryIce = findRamp(7,8);
                    break;
                case 8:
                    // climb up the ramp
                    stateDryIce = climbRamp(8,9);
                    break;
                default:
                    stop();
                    // error

            }
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
        enableLED = false;
        sensorRGB.enableLed(enableLED);
        sensorODSLeft.enableLed(enableLED);
        sensorODSRight.enableLed(enableLED);
        sensorODSLeft.close();
        sensorODSRight.close();
        sensorRGB.close();
        sensorGyro.close();
        telemetry.addData("STATE", "Ended");
    }

    int goStraightToCenterLine(int startState, int endState, float power, long timeLimit) {

        int retCode = startState;

        char color = ResQUtils.getColor(sensorRGB, colorSensitivity, minColorBrightness, rgb);
        telemetry.addData("COLOR", ": " + color +
                " r=" + String.format("%d", rgb.r) +
                " g=" + String.format("%d", rgb.g) +
                " b=" + String.format("%d", rgb.b));

        int distanceWheel = getOdometer();
        telemetry.addData("ODOMETER", "encoder: " + String.format("%05d", distanceWheel));

        if ( distanceWheel > StarLineToCenterLineDistance * 0.5
                && (color == 'b' || color == 'r')) {
            retCode = endState;
        } else {
            retCode = goStraight(startState, endState, power, StarLineToCenterLineDistance, timeLimit);
        }

        if (endState == retCode) {
            // set the next stateDryIce
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
                targetAngle = normalizeAngle(targetAngle - turnAngle);
            } else {
                telemetry.addData("STATE", ": Turning left...");
                targetAngle = normalizeAngle(targetAngle + turnAngle);
            }
        } else {
            telemetry.addData("ACTION", "Moving to center line");
        }
        return retCode;
    }

    int goStraightFromCenterlineToBeaconLine(int startState, int endState, float power, long timeLimit) {
        int retCode = goStraight(startState, endState, power, CenterlineToBeaconLineDistance, timeLimit);

        char color = ResQUtils.getColor(sensorRGB, colorSensitivity, minColorBrightness, rgb);
        telemetry.addData("COLOR", ": " + color +
                " r=" + String.format("%d", rgb.r) +
                " g=" + String.format("%d", rgb.g) +
                " b=" + String.format("%d", rgb.b));
        if (color == teamColor) {
            retCode = endState;
        }

        if (retCode == endState) {

            leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
            rightWheelStartPos = motorBottomRight.getCurrentPosition();

            // set the next stateDryIce
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
                targetAngle = normalizeAngle(targetAngle - (90-turnAngle));
            } else {
                telemetry.addData("STATE", ": Turning left...");
                targetAngle = normalizeAngle(targetAngle + (90-turnAngle));
            }
        } else {
            telemetry.addData("ACTION", "Moving from center line to beacon line");
        }
        return retCode;
    }

    int goStraightFromBeaconlineToBeacon(int startState, int endState, float power, long timeLimit) {

        int distanceLeft = sensorODSLeft.getLightDetectedRaw();
        int distanceRight = sensorODSRight.getLightDetectedRaw();
        int retCode = startState;

        telemetry.addData("STATE",
                ": Move toward beacon...distance ("
                        + String.format("%03d", distanceLeft) + ", "
                        + String.format("%03d", distanceRight) + ") ");

        if (distanceLeft < collisionDistThreshold && distanceRight < collisionDistThreshold) {
            retCode = goStraight(startState, endState, power, BeaconLineToBeaconDistance, timeLimit);
        }
        else {
            retCode = endState;
        }

        if (retCode == endState) {
            motorBottomLeft.setPower(0.0);
            motorBottomRight.setPower(0.0);
            leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
            rightWheelStartPos = motorBottomRight.getCurrentPosition();
            telemetry.addData("STATE", "Move from beacon line to beacon completed");
        } else {
            telemetry.addData("ACTION", "Moving from beacon line to beacon");
        }

        return retCode;
    }

    int goStraightFromCenterlineToRampLine(int startState, int endState, float power, long timeLimit) {
        int retCode = goStraight(startState, endState, power, CenterlineToBeaconLineDistance, timeLimit);

        if (retCode == endState) {
            // set the next stateDryIce
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
                targetAngle = normalizeAngle(targetAngle - 45);
            } else {
                telemetry.addData("STATE", ": Turning left...");
                targetAngle = normalizeAngle(targetAngle + 45);
            }
        }
        return retCode;
    }

    int goStraightFromRampToBeanconLine(int startState, int endState, float power, long timeLimit) {
        int retCode = goStraight(startState, endState, power, RampLineToBeaconLineDistance, timeLimit);

        if (retCode == endState) {
            // set the next stateDryIce
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
                targetAngle = normalizeAngle(targetAngle - 90);
            } else {
                telemetry.addData("STATE", ": Turning left...");
                targetAngle = normalizeAngle(targetAngle + 90);
            }
        }
        return retCode;
    }

    int goStraightBeaconLine(int startState, int endState, float power, long timeLimit) {
        int retCode = goStraight(startState, endState, power, CenterlineToBeaconLineDistance, timeLimit);

        if (retCode == endState) {
            // set the next stateDryIce
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
                targetAngle = normalizeAngle(targetAngle - 45);
            } else {
                telemetry.addData("STATE", ": Turning left...");
                targetAngle = normalizeAngle(targetAngle + 45);
            }
        }
        return retCode;
    }

    int moveToBeacon(double power, int startState, int endState) {
        int stateCode = startState;

        int distanceLeft = sensorODSLeft.getLightDetectedRaw();
        int distanceRight = sensorODSRight.getLightDetectedRaw();
        int distanceWheel = Math.min(Math.abs(leftWheelCurrent - leftWheelStartPos), Math.abs(rightWheelCurrent - rightWheelStartPos));
        char color = ResQUtils.getColor(sensorRGB, colorSensitivity, minColorBrightness, rgb);

        telemetry.addData("STATE",
                ": Move toward beacon...distance ("
                        + String.format("%03d", distanceLeft) + ", "
                        + String.format("%03d", distanceRight) + ") ");
        telemetry.addData("COLOR", ": " + color +
                " r=" + String.format("%d", rgb.r) +
                " g=" + String.format("%d", rgb.g) +
                " b=" + String.format("%d", rgb.b));
        if (System.currentTimeMillis() - lastStateTimeStamp < 1500) {
            maintainAngle(targetAngle, currentGyro, power); // move away from color zone
        } else {
            if (distanceWheel < CenterlineToBeaconLineDistance
                && distanceLeft < collisionDistThreshold
                && distanceRight < collisionDistThreshold
                && color != teamColor) {
                maintainAngle(targetAngle, currentGyro, searchPower);
            } else {
                motorBottomRight.setPower(0.0);
                motorBottomLeft.setPower(0.0);
                lastStateTimeStamp = System.currentTimeMillis();
                telemetry.addData("STATE", ": Move to beacon done");
                leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
                rightWheelStartPos = motorBottomRight.getCurrentPosition();
                stateCode = endState;
            }
        }
        return stateCode;
    }

    int searchBeacon(float power, int startState, int endState) {

        int retCode = startState;
        int distanceLeft = sensorODSLeft.getLightDetectedRaw();
        int distanceRight = sensorODSRight.getLightDetectedRaw();
        telemetry.addData("STATE",
                ": Search beacon...distance ("
                        + String.format("%03d", distanceLeft) + ", "
                        + String.format("%03d", distanceRight) + ") ");
        if (distanceLeft < collisionDistThreshold
        && distanceRight < collisionDistThreshold) {
            retCode = goStraight(startState, endState, power, BeaconLineToBeaconDistance, 30000);
        }

        if ( endState == retCode) {
            motorBottomRight.setPower(0.0);
            motorBottomLeft.setPower(0.0);
            lastStateTimeStamp = System.currentTimeMillis();
            telemetry.addData("STATE", ": Search beacon done");
            leftWheelStartPos = motorBottomLeft.getCurrentPosition();
            rightWheelStartPos = motorBottomRight.getCurrentPosition();
        }

        return retCode;
    }

    int dropClimber (int startState, int endState) {
        int stateCode = startState;
        stateCode = endState;
        return stateCode;
    }

    int touchButton(int startState, int endState) {
        int stateCode = startState;
        stateCode = endState;
        return stateCode;
    }

    int backup(int startState, int endState) {
        int stateCode = startState;
        stateCode = endState;
        return stateCode;
    }

    int findRamp(int startState, int endState) {
        int stateCode = startState;
        stateCode = endState;
        return stateCode;
    }

    int climbRamp(int startState, int endState) {
        int stateCode = startState;
        stateCode = endState;
        return stateCode;
    }

    void turnLeft(double power) {
        motorBottomRight.setPower(power);
        motorBottomLeft.setPower(-power);
    }

    void turnRight(double power) {
        motorBottomRight.setPower(-power);
        motorBottomLeft.setPower(power);
    }

    double getDeltaPowerByDeltaAngle(float deltaAngle, float gain) {
        if (Math.abs(deltaAngle) < targetAngleTolerance) {
            return 0.0f;
        } else {
            return ResQUtils.lookUpTableFunc(deltaAngle * gain, angle2PowerLUT); // 180 degree max
        }
    }

    float getDeltaDistanceByDeltaAngle(float deltaAngle, float gain) {
        if (Math.abs(deltaAngle) < targetAngleTolerance) {
            return 0.0f;
        } else {
            return ResQUtils.lookUpTableFunc(deltaAngle * gain , angle2DistanceLUT); // 180 degree max
        }
    }

    float maintainAngle(float target, float current, double power) {
        float skew = getAngleDelta((int)current, (int)target);
        double turn = getDeltaPowerByDeltaAngle(skew, targetAngleTolerance*0.0056f);

        // increase turn power if stuck
        if ( Math.abs(skew - lastSkew) < targetAngleTolerance) {
            skewPowerScale *= skewPowerGain;
        }
        else {
            skewPowerScale = 1.0f;
        }
        turn *= skewPowerScale;

        // turn
        double left = Range.clip(power + turn, -1.0, 1.0);
        double right = Range.clip(power -turn, -1.0, 1.0);
        motorBottomRight.setPower(left);
        motorBottomLeft.setPower(right);
        telemetry.addData("SKEW", String.format("%.2g", skew) + "degree");
        telemetry.addData("WHEEL", "left pwr:" + String.format("%.2g", left) +
                " right pwr:" + String.format("%.2g", right) +
                " turn pwr:" + String.format("%.2g", turn));
        lastSkew = skew;
        return skew;
    }

    float maintainAngleByEncoder(float targetAngle, float currentAngle, float speed, float power) {
        float skew = getAngleDelta((int)currentAngle, (int)targetAngle);
        float turn = getDeltaDistanceByDeltaAngle(skew, 0.005f);

        // turn
        float left  = speed + turn;
        float right = speed - turn;
        moveLeftWheelByEncoder((int) (leftWheelCurrent + left), power);
        moveRightWheelByEncoder((int) (rightWheelCurrent + right), power);

        telemetry.addData("SKEW", String.format("%.2g", skew) + "degree");
        telemetry.addData("WHEEL", "left distance:" + String.format("%.2g", left) +
                " right distance:" + String.format("%.2g", right) +
                " power:" + String.format("%.2g", power));
        lastSkew = skew;
        return skew;
    }


    /**
     *
     * @param angle
     * @return [0,360)
     */
    int normalizeAngle (int angle) {
        int ret = angle%360;
        if (ret<0) {
            ret = 360+ret;
        }
        return ret;
    }

    int getAngleDelta (int from, int to) {
        int delta = (to - from)%360;
        if (delta >180) {
            delta = delta - 360;
        }
        else if (delta < -180)
        {
            delta = 360+delta;
        }
        return delta;
    }

    int goStraight(int startState, int endState, float power, int distanceLimit, long timeLimit) {

        int distanceWheel = getOdometer();
        telemetry.addData("ODOMETER", "encoder: " + String.format("%05d", distanceWheel));

        if (distanceWheel > distanceLimit
                || System.currentTimeMillis() - lastStateTimeStamp > timeLimit) { // time out
            // stop at the center blue/red line
            motorBottomRight.setPower(0);
            motorBottomLeft.setPower(0);
            lastStateTimeStamp = System.currentTimeMillis();
            leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
            rightWheelStartPos = motorBottomRight.getCurrentPosition();
            return endState;
        } else {
            telemetry.addData("STATE", ": Going Straight...");
            if (System.currentTimeMillis() - lastStateTimeStamp < 1000) { // first second, fast
                maintainAngle(targetAngle, currentGyro, power);
            } else {  // search slowly to prevent miss the b/r tapes
                maintainAngle(targetAngle, currentGyro, searchPower);
            }
        }
        return startState;
    }

    int turn(int startState, int endState, double power, int currentAngle, int stopAngle) {

        if (Math.abs(currentAngle - stopAngle) < targetAngleTolerance) {
            telemetry.addData("STATE", ": Turn done");
            motorBottomRight.setPower(0.0);
            motorBottomLeft.setPower(0.0);
            lastStateTimeStamp = System.currentTimeMillis();
            leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
            rightWheelStartPos = motorBottomRight.getCurrentPosition();
            return endState;
        } else {
            maintainAngle(stopAngle, currentAngle, power);
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
            } else {
                telemetry.addData("STATE", ": Turning left...");
            }
        }

        return startState;
    }

    int getOdometer() {
        return (motorBottomLeft.getCurrentPosition()-leftWheelStartPos
                +motorBottomRight.getCurrentPosition()-rightWheelStartPos)/2;
    }
}

