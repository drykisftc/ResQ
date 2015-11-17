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

import android.renderscript.Element;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Queue;
import java.util.Vector;
import java.util.Collections;

class TurnData implements Comparable<TurnData>
{
    int intensity = -1;
    double turnPower = 0.0;
    double gyroPos = 0.0;

    public int compareTo (TurnData a)
    {
        if (turnPower > a.turnPower)
            return 1;
        else
            return 0;
    }
}

/**
 * Auto Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ResQAuto extends ResQTeleOp {

    ColorSensor sensorRGB;

    OpticalDistanceSensor sensorODSLeft;
    OpticalDistanceSensor sensorODSRight;

    GyroSensor sensorGyro;

    ResQCamera camera;

    boolean enableLED = true;

    float colorSensitivity = 1.5f;

    char teamColor = 'b';

    long startTime = 0;

    long timeBudget = 30000; // 30 seconds
    long lastStateTimeStamp = 0;

    float cruisePower = 0.3f;
    float searchPower = 0.25f;
    float turnPower = 0.2f;

    int collisionDistThreshold = 20;
    int minColorBrightness = 3;
    TurnData prevTurnData;
    double prevTurnPower = 0.0;

    int refGyro = 0;
    int prevGyro = 0;
    int currentGyro = 0;
    int targetAngle = 0;
    int targetAngleTolerance = 1;
    float[] angle2PowerLUT = {0.05f, 0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.5f, 0.6f};
    float lastSkew =0;
    float skewPowerScale = 1.0f;
    float skewPowerGain = 1.06f;

    GyroData gyroData;
    int refXRotation = 0;
    int refYRotation = 0;
    int refZRotation = 0;

    RGB rgb= new RGB(0,0,0);;

    Queue<TurnData> turnDataFIFO;

    /**
     * Constructor
     */
    public ResQAuto() {

    }

    int[] stateArray = {0, // init
            1, // go straight
            2, // turn
            3, // find the beacon
            4, // touch the button
            5, // backup
            6,// find the rampe with correct color
            7,// climb up the ramp
            8}; // end
    int state = -1;

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

        // wait 1 second for gyro calibration
        try {
            Thread.sleep(1500);                 //1000 milliseconds is one second.
        } catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        state = 0;
        telemetry.addData("TEAM", "Team color:" + teamColor);
        telemetry.addData("STATE", "state: Init done");
        //currentGyro = getGyroHeading();
        //upateRotationData();
        ResQUtils.getGyroData(sensorGyro, gyroData);
        currentGyro = gyroData.heading;
        refGyro = currentGyro;
        prevGyro = currentGyro;
        telemetry.addData("GYRO", "GYRO: " + String.format("%03d", normalizeAngle(currentGyro-refGyro))
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

        // set the next state
        targetAngle = currentGyro;
        startTime = System.currentTimeMillis();

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
            //currentGyro = getGyroHeading();
            //upateRotationData();
            ResQUtils.getGyroData(sensorGyro, gyroData);
            currentGyro = gyroData.heading;
            telemetry.addData("GYRO", "GYRO: " + String.format("%03d", targetAngle)
                    + " (" + String.format("%03d", currentGyro)
                    + " ," + String.format("%03d", gyroData.xRotation)
                    + " ," + String.format("%03d", gyroData.yRotation)
                    + " ," + String.format("%03d", gyroData.zRotation)+")");
            switch (state) {
                case 0:
                    // go straight
                    state = goStraight(cruisePower, 3500);
                    break;
                case 1:
                    // turn
                    state = turn(turnPower);
                    break;
                case 2:
                    state = moveToBeacon(cruisePower);
                    break;
                case 3:
                    // find the beacon
                    state = searchBeacon(searchPower);
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
    }

    int goStraight(double power, long timeLimit) {
        int stateCode = 0;
        char color = ResQUtils.getColor(sensorRGB, colorSensitivity, minColorBrightness,rgb);
        telemetry.addData("COLOR", ": " + color +
                " r=" + String.format("%d", rgb.r) +
                " g=" + String.format("%d", rgb.g) +
                " b=" + String.format("%d", rgb.b));

        if (color == 'b' || color == 'r'
                || System.currentTimeMillis() - startTime > timeLimit) { // time out
            // stop at the center blue/red line
            motorBottomRight.setPower(0);
            motorBottomLeft.setPower(0);
            lastStateTimeStamp = System.currentTimeMillis();
            // set the next state
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
                targetAngle = normalizeAngle(targetAngle - 90);
            } else {
                telemetry.addData("STATE", ": Turning left...");
                targetAngle = normalizeAngle(targetAngle + 90);
            }
            stateCode = 1;
        } else {
            telemetry.addData("STATE", ": Going Straight...");
            if (System.currentTimeMillis() - startTime < 1000) { // first second, fast
                maintainAngle(targetAngle, currentGyro, power);
            } else {  // search slowly to prevent miss the b/r tapes
                maintainAngle(targetAngle, currentGyro, searchPower);
            }
        }
        return stateCode;
    }

    int turn(double power) {
        int stateCode = 1;

        if (Math.abs(currentGyro - targetAngle) < targetAngleTolerance) {
            motorBottomRight.setPower(0.0);
            motorBottomLeft.setPower(0.0);
            lastStateTimeStamp = System.currentTimeMillis();
            telemetry.addData("STATE", ": Turn done");
            stateCode = 2;
        } else {
            maintainAngle(targetAngle, currentGyro, 0.0);
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
            } else {
                telemetry.addData("STATE", ": Turning left...");
            }
        }

        return stateCode;
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
            if (distanceLeft < collisionDistThreshold
                && distanceRight < collisionDistThreshold
                && color != teamColor) {
                maintainAngle(targetAngle, currentGyro, searchPower);
            } else {
                motorBottomRight.setPower(0.0);
                motorBottomLeft.setPower(0.0);
                lastStateTimeStamp = System.currentTimeMillis();
                telemetry.addData("STATE", ": Move to beacon done");
                stateCode = 3;
            }
        }

        return stateCode;
    }

    int searchBeacon(double power) {
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

}

