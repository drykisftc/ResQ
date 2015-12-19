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

    ResQCamera camera;

    boolean enableLED = true;

    float colorSensitivity = 1.25f;

    char teamColor = 'b';

    long startTime = 0;

    long timeBudget = 30000; // 30 seconds
    long lastStateTimeStamp = 0;

    float cruisePower = 0.99f;
    float searchPower = 0.5f;
    float turnPower = 0.4f;

    int collisionDistThreshold = 50;
    int minColorBrightness = 8;
    TurnData prevTurnData;
    double prevTurnPower = 0.0;

    int refGyro = 0;
    int prevGyro = 0;
    int currentGyro = 0;
    int targetAngle = 0;
    int targetAngleTolerance = 2;
    float[] angle2PowerLUT = {0.0005f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f,
                              0.06f, 0.07f, 0.08f, 0.09f, 0.1f,  0.11f,
                              0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f,
                              0.18f,0.19f, 0.2f, 0.21f, 0.22f, 0.23f,
                              0.24f,0.25f, 0.275f, 0.3f, 0.325f, 0.35f,
                              0.375f, 0.4f, 0.425f, 0.45f, 0.5f, 0.6f};
    float[] angle2DistanceLUT = {0.00f, 5.1f, 10.15f, 15.2f, 20.25f, 30.3f, 40.35f, 50.4f};
    float lastSkew =0;
    float skewPowerScale = 1.0f;
    float skewPowerGain = 1.19f;

    GyroData gyroData;
    int refXRotation = 0;
    int refYRotation = 0;
    int refZRotation = 0;

    RGB rgb= new RGB(0,0,0);

    Queue<TurnData> turnDataFIFO;

    int StarLineToCenterLineDistance = 11455;
    int CenterlineToBeaconLineDistance = 5728;
    int BeaconLineToBeaconDistance = 2864;

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

        camera = new ResQCamera();

        motorBottomLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBottomRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

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

    }

    public void start() {

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
                    stateDryIce = goStraightToCenterLine(cruisePower, 8000);
                    break;
                case 1:
                    // turn
                    stateDryIce = turn(1,2,turnPower, currentGyro, targetAngle);
                    break;
                case 2:
                    stateDryIce = moveToBeacon(cruisePower);
                    break;
                case 3:
                    // find the beacon
                    stateDryIce = searchBeacon(searchPower);
                    break;
                case 4:
                    // touch the button
                    touchButton();
                    break;
                case 5:
                    // backup
                    backup();
                    break;
                case 6:
                    // find the ramp with correct color
                    findRamp();
                    break;
                case 7:
                    // climb up the ramp
                    climbRamp();
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
        sensorODSLeft.enableLed(enableLED);
        telemetry.addData("STATE", "Ended");
    }

    int goStraightToCenterLine(float power, long timeLimit) {
        int retCode = goStraight(0, 1, power, StarLineToCenterLineDistance, timeLimit);

        if (retCode == 1) {
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

    int turn(int startState, int endState, double power, int currentAngle, int stopAngle) {

        if (Math.abs(currentAngle - stopAngle) < targetAngleTolerance) {
            motorBottomRight.setPower(0.0);
            motorBottomLeft.setPower(0.0);
            lastStateTimeStamp = System.currentTimeMillis();
            telemetry.addData("STATE", ": Turn done");
            leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
            rightWheelStartPos = motorBottomRight.getCurrentPosition();
            return endState;
        } else {
            maintainAngle(stopAngle, currentAngle, 0.0);
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
            } else {
                telemetry.addData("STATE", ": Turning left...");
            }
        }

        return startState;
    }

    int moveToBeacon(double power) {
        int stateCode = 2;

        // stop
//        motorBottomRight.setPower(0.0);
//        motorBottomLeft.setPower(0.0);
//        camera.snapPicture();

        // get skew angle from the camera


        // adjust targe angle

        // keep the beacon in center until ODS trigger
        //if ( sensorODS.getLightDetectedRaw() )
        int distanceLeft = sensorODSLeft.getLightDetectedRaw();
        int distanceRight = sensorODSRight.getLightDetectedRaw();
        int distanceWheel = Math.min(leftWheelCurrent-leftWheelStartPos, rightWheelCurrent-rightWheelStartPos);
        char color = ResQUtils.getColor(sensorRGB, colorSensitivity, minColorBrightness,rgb);

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
                stateCode = 3;
            }
        }
        return stateCode;
    }

    int searchBeacon(float power) {

        int distanceLeft = sensorODSLeft.getLightDetectedRaw();
        int distanceRight = sensorODSRight.getLightDetectedRaw();
        telemetry.addData("STATE",
                ": Search beacon...distance ("
                        + String.format("%03d", distanceLeft) + ", "
                        + String.format("%03d", distanceRight) + ") ");
        if (distanceLeft < collisionDistThreshold
        && distanceRight < collisionDistThreshold) {
            return goStraight(3, 4, power, BeaconLineToBeaconDistance, 3000);
        }
        return 4;
    }

    int searchBeacon2(double power) {
        int stateCode = 3;

        // stop and take picture
//        motorBottomRight.setPower(0.0);
//        motorBottomLeft.setPower(0.0);
//        camera.snapPicture();

        // get skew angle from the camera

        // get distance sensor
        int distanceLeft = sensorODSLeft.getLightDetectedRaw();
        int distanceRight = sensorODSRight.getLightDetectedRaw();
        char color = ResQUtils.getColor(sensorRGB, colorSensitivity, minColorBrightness,rgb);

        telemetry.addData("STATE",
                ": Search beacon...distance ("
                        + String.format("%03d", distanceLeft) + ", "
                        + String.format("%03d", distanceRight) + ") ");
        telemetry.addData("COLOR", ": " + color +
                " r=" + String.format("%d", rgb.r) +
                " g=" + String.format("%d", rgb.g) +
                " b=" + String.format("%d", rgb.b));

        // following color lines, searching for the white line
        if (System.currentTimeMillis() - lastStateTimeStamp < 5500) {
            ResQUtils.followColorLine('u', sensorRGB,
                    colorSensitivity, minColorBrightness,
                    motorBottomLeft, motorBottomRight, searchPower, turnPower);
        }
        else {
            motorBottomRight.setPower(0.0);
            motorBottomLeft.setPower(0.0);
            lastStateTimeStamp = System.currentTimeMillis();
            telemetry.addData("STATE", ": Search beacon done");
            stateCode = 4;
        }

        return stateCode;
    }

    int touchButton() {
        int stateCode = 4;
        stateCode = 5;
        return stateCode;
    }

    int backup() {
        int errorCode = 5;
        errorCode = 6;
        return errorCode;
    }

    int findRamp() {
        int errorCode = 6;
        errorCode = 7;
        return errorCode;
    }

    int climbRamp() {
        int errorCode = 7;
        errorCode = 8;
        return errorCode;
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
        double turn = getDeltaPowerByDeltaAngle(skew, 0.005f);

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
        char color = ResQUtils.getColor(sensorRGB, colorSensitivity, minColorBrightness,rgb);
        telemetry.addData("COLOR", ": " + color +
                " r=" + String.format("%d", rgb.r) +
                " g=" + String.format("%d", rgb.g) +
                " b=" + String.format("%d", rgb.b));

        int distanceWheel = getOdometer();

        if (distanceWheel > distanceLimit
                || (distanceWheel > distanceLimit*0.5 && (color == 'b' || color == 'r'))
                || System.currentTimeMillis() - startTime > timeLimit) { // time out
            // stop at the center blue/red line
            motorBottomRight.setPower(0);
            motorBottomLeft.setPower(0);
            leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
            rightWheelStartPos = motorBottomRight.getCurrentPosition();
            lastStateTimeStamp = System.currentTimeMillis();

            return endState;
        } else {
            telemetry.addData("STATE", ": Going Straight...");
            if (System.currentTimeMillis() - startTime < 1000) { // first second, fast
                maintainAngle(targetAngle, currentGyro, power);
            } else {  // search slowly to prevent miss the b/r tapes
                maintainAngle(targetAngle, currentGyro, searchPower);
            }
        }
        return startState;
    }

    int getOdometer() {
        return Math.min(leftWheelCurrent-leftWheelStartPos, rightWheelCurrent-rightWheelStartPos);
    }
}

