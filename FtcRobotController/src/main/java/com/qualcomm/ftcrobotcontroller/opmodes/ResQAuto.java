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

    float colorSensitivity = 1.3f;

    char teamColor = 'b';

    long startTime = 0;

    long timeBudget = 30000; // 30 seconds
    long lastStateTimeStamp = 0;

    double cruisePower = 0.4;
    double searchPower = 0.25;
    double turnPower = 0.2;

    int collisionDistThreshold = 20;
    int minColorBrightness = 10;
    TurnData prevTurnData;
    double prevTurnPower = 0.0;

    int refGyro = 0;
    int prevGyro = 0;
    int currentGyro = 0;
    int targetAngle = 0;
    int targetAngleTolerance = 1;
    float[] angle2PowerLUT = {0.2f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.5f};

    int heading = 0;
    int xRotation=0;
    int yRotation=0;
    int zRotation=0;
    int refXRotation = 0;
    int refYRotation = 0;
    int refZRotation = 0;

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


        sensorODSLeft = hardwareMap.opticalDistanceSensor.get("leftODS");
        sensorODSLeft.enableLed(true);
        sensorODSRight = hardwareMap.opticalDistanceSensor.get("rightODS");
        sensorODSRight.enableLed(true);

        sensorRGB = hardwareMap.colorSensor.get("armColor");
        sensorRGB.enableLed(true);

        sensorGyro = hardwareMap.gyroSensor.get("headGYRO");
        sensorGyro.calibrate();

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
        getGyroData();
        currentGyro = heading;
        refGyro = currentGyro;
        prevGyro = currentGyro;
        telemetry.addData("GYRO", "GYRO: " + String.format("%03d", normalizeAngle(currentGyro-refGyro))
                + " (" + String.format("%03d", currentGyro)
                + " ," + String.format("%03d", xRotation)
                + " ," + String.format("%03d", yRotation)
                + " ," + String.format("%03d", zRotation)+")");

        sensorRGB.enableLed(true);

        sensorODSLeft.enableLed(true);
        sensorODSRight.enableLed(true);

    }

    public void start() {

        //currentGyro = getGyroHeading();
        //upateRotationData();
        getGyroData();
        currentGyro = heading;
        refGyro = currentGyro;
        prevGyro = currentGyro;
        refXRotation = xRotation;
        refYRotation = yRotation;
        refZRotation = zRotation;

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
            getGyroData();
            currentGyro = heading;
            telemetry.addData("GYRO", "GYRO: " + String.format("%03d", targetAngle)
                    + " (" + String.format("%03d", currentGyro)
                    + " ," + String.format("%03d", xRotation)
                    + " ," + String.format("%03d", yRotation)
                    + " ," + String.format("%03d", zRotation)+")");
            switch (state) {
                case 0:
                    // go straight
                    state = goStraight(cruisePower);
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
        sensorRGB.enableLed(false);
        sensorODSLeft.enableLed(false);
        sensorODSLeft.enableLed(false);
    }

    int goStraight(double power) {
        int stateCode = 0;
        char color = getColor(colorSensitivity);
        telemetry.addData("COLOR", ": " + color);

        if (color == 'b' || color == 'r') {
            // stop at the center blue/red line
            motorBottomRight.setPower(0);
            motorBottomLeft.setPower(0);
            lastStateTimeStamp = System.currentTimeMillis();
            // set the next state
            if (teamColor == 'b') {
                telemetry.addData("STATE", ": Turning right...");
                targetAngle = normalizeAngle(currentGyro - 90);
            } else {
                telemetry.addData("STATE", ": Turning left...");
                targetAngle = normalizeAngle(currentGyro + 90);
            }
            return 1;
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

        // TODO: turn 90 degree until camera see beacon
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

        // keep the beacon in center until ODS trigger
        //if ( sensorODS.getLightDetectedRaw() )
        int distanceLeft = sensorODSLeft.getLightDetectedRaw();
        int distanceRight = sensorODSLeft.getLightDetectedRaw();
        char color = getColor(colorSensitivity);
        telemetry.addData("STATE",
                ": Move toward beacon...distance ("
                        + String.format("%03d", distanceLeft)+ ", "
                        + String.format("%03d", distanceRight) + ") "
                        + " Color: " + color);
        if (distanceLeft < collisionDistThreshold
                && distanceRight < collisionDistThreshold
                && color != teamColor) {
            if (System.currentTimeMillis() - lastStateTimeStamp < 1000) {
                maintainAngle(targetAngle, currentGyro, power);
            } else {
                maintainAngle(targetAngle, currentGyro, searchPower);
            }
        } else {
            motorBottomRight.setPower(0.0);
            motorBottomLeft.setPower(0.0);
            lastStateTimeStamp = System.currentTimeMillis();
            telemetry.addData("STATE", ": Move to beacon done");
            stateCode = 3;
        }

        return stateCode;
    }

    int searchBeacon(double power) {
        int stateCode = 3;

        // following color lines, searching for the white line
        stateCode = 4;

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

    char getColor(float snrLimit) {
        int r = sensorRGB.red();
        int g = sensorRGB.green();
        int b = sensorRGB.blue();

        telemetry.addData("RGB", "r=" + String.format("%d", r) +
                "g=" + String.format("%d", g) +
                "b=" + String.format("%d", b));

        // find the max
        int m = Math.max(r, g);
        m = Math.max(m, b);
        int sum = r + g + b;

        // if SNR is good
        if (sum > minColorBrightness && m > sum * 0.333 * snrLimit) {
            if (m == g) return 'g';
            if (m == r) return 'r';
            if (m == b) return 'b';
        }
        return 'u'; // unknown color
    }

    void turnLeft(double power) {
        motorBottomRight.setPower(power);
        motorBottomLeft.setPower(-power);
    }

    void turnRight(double power) {
        motorBottomRight.setPower(-power);
        motorBottomLeft.setPower(power);
    }

    int getColorIntensity(char color) {
        int intensity = 0;
        switch (color) {
            case 'r':
                intensity = sensorRGB.red();
                break;
            case 'b':
                intensity = sensorRGB.blue();
                break;
            case 'g':
                intensity = sensorRGB.green();
                break;
            case 'w':
                intensity = sensorRGB.red() + sensorRGB.green() + sensorRGB.blue();
                break;
            default:
                break;
        }
        return intensity;
    }

    void followColor(char color,
                     double powerForward, double maxPowerTurn,
                     int intensityThreshold, int intensityDeltaThreshold) {

        int intensity = getColorIntensity(color);

        // totally lost, make circles until find the line again
        if (intensity < intensityThreshold) {
            turnDataFIFO.clear();
            motorBottomRight.setPower(maxPowerTurn);
            motorBottomLeft.setPower(-maxPowerTurn);
            return;
        }

        // adjust turn power
        double powerTurn = 0.0;
        if (prevTurnData.intensity > 0) {

            int colorDelta = intensity - prevTurnData.intensity;
            if (Math.abs(colorDelta) > intensityDeltaThreshold) {
                powerTurn = estimateTurnPower(maxPowerTurn);
            }
        }

        // adjust motor power
        motorBottomRight.setPower(powerForward + powerTurn);
        motorBottomLeft.setPower(powerForward - powerTurn);

        // log history
        prevTurnData.intensity = intensity;
        prevTurnData.turnPower = prevTurnPower;
        if (turnDataFIFO.size() >= 3) {
            turnDataFIFO.remove();
        }
        turnDataFIFO.offer(prevTurnData);
        prevTurnPower = turnPower;
    }

    double estimateTurnPower(double maxTurnPower) {
        double p = maxTurnPower;

        //copy turnDataFIFO
        Vector<TurnData> v = new Vector<TurnData>(turnDataFIFO.size());
        for (TurnData q : turnDataFIFO) {
            v.add(q);
        }

        // add up deltaPower
        for (int i = 1; i < v.size(); i++) {
            v.elementAt(i).turnPower += v.elementAt(i - 1).turnPower;
        }

        // sort power
        Collections.sort(v);

        //  3 point interpolation, quadratic fit to maximize the intensity


        return p;
    }

    double getDeltaPowerByDeltaAngle(int deltaAngle) {
        if (Math.abs(deltaAngle) < targetAngleTolerance) {
            return 0.0f;
        } else {
            return ResQUtils.lookUpTableFunc((float) deltaAngle * 0.005f, angle2PowerLUT); // 180 degree max
        }
    }

    void maintainAngle(int target, int current, double power) {
        int skew = getAngleDelta(current, target);
        double turn = getDeltaPowerByDeltaAngle(skew);
        double left = power + turn;
        double right = power - turn;
        motorBottomRight.setPower(left);
        motorBottomLeft.setPower(right);
        telemetry.addData("SKEW", String.format("%03d", skew) + "degree");
        telemetry.addData("WHEEL", "left pwr:" + String.format("%.2g", left) +
                " right pwr:" + String.format("%.2g", right) +
                " turn pwr:" + String.format("%.2g", turn));
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

    void getGyroData (){
        heading = sensorGyro.getHeading();
        xRotation = sensorGyro.rawX();
        yRotation = sensorGyro.rawY();
        zRotation = sensorGyro.rawZ();
    }
}

