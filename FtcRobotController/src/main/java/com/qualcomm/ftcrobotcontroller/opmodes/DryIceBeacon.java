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
public class DryIceBeacon extends DryIceAuto {

    int dumpClimberArmPosition = 0;
    int dumpArmDelta = 2900;
    int minWhiteLineBrightness = 10;

    @Override
    public void init() {
        super.init();
        teamColor = 'b';
        StarLineToCenterLineDistance = 4900;
        CenterlineToBeaconLineDistance = 9800;
        BeaconLineToBeaconDistance = 2800;
        RampLineToBeaconLineDistance = 3500;
        turnAngle = 45;
    }

    public void start () {
        super.start();
        dumpClimberArmPosition = rightArmUpperLimit + dumpArmDelta;
    }

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
                    + " ," + String.format("%03d", gyroData.zRotation) + ")");
            telemetry.addData("DISTANCE", "Left:" + String.format("%05d", leftWheelCurrent - leftWheelStartPos)
                    + ". Right:" + String.format("%05d", rightWheelCurrent - rightWheelStartPos));

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
                    stateDryIce = goStraightFromCenterlineToBeaconLine(2, 3, scoutPower, 20000); // should finish in 20 sec
                    break;
                case 3:
                    // turn
                    stateDryIce = turn(3, 4, 0.02f, currentGyro, targetAngle);
                    break;
                case 4:
                    stateDryIce = goStraightFromBeaconlineToBeacon(4, 5, searchPower, 25000);
                    break;
                case 5:
                    // touch the button
                    stateDryIce = dropClimber(5, 6);
                    break;
                case 6:
                    // touch the button
                    stateDryIce = touchButton(6, 7);
                    break;
                case 7:
                    // backup
                    stateDryIce = backup(7, 8);
                    break;
                case 8:
                    // find the ramp with correct color
                    stateDryIce = findRamp(8, 9);
                    break;
                case 9:
                    // climb up the ramp
                    stateDryIce = climbRamp(9, 10);
                    break;
                default:
                    stop();
                    // error

            }
        }
    }

    int goStraightFromCenterlineToBeaconLine(int startState, int endState, float power, long timeLimit) {
        int retCode = startState;

        char color = ResQUtils.getColor(sensorRGB, colorSensitivity, minColorBrightness, rgb);
        telemetry.addData("COLOR", ": " + color +
                " r=" + String.format("%d", rgb.r) +
                " g=" + String.format("%d", rgb.g) +
                " b=" + String.format("%d", rgb.b));
        if (color == 'u' && rgb.r+rgb.g+rgb.b >= minWhiteLineBrightness) {
            leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
            rightWheelStartPos = motorBottomRight.getCurrentPosition();
            retCode = goStraight(startState, endState, power, 400, timeLimit);;
        }
        else if (color == teamColor) {
            // lower speed
            leftWheelStartPos =  motorBottomLeft.getCurrentPosition();
            rightWheelStartPos = motorBottomRight.getCurrentPosition();
            retCode = goStraight(startState, endState, power*0.5f, BeaconLineToBeaconDistance/3, timeLimit);
        }
        else {
            retCode = goStraight(startState, endState, power, CenterlineToBeaconLineDistance, timeLimit);
        }

        if (retCode == endState) {
            motorBottomLeft.setPower(0.0);
            motorBottomRight.setPower(0.0);
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


    int dropClimber (int startState, int endState) {
        int stateCode = startState;

        int armPosition = motorTopRight.getCurrentPosition();

        if ( Math.abs(armPosition-dumpClimberArmPosition) > dumpArmDelta/2) {
            if (motorTopRight.getMode() != DcMotorController.RunMode.RUN_TO_POSITION) {
                motorTopRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            }
            motorTopRight.setTargetPosition(dumpClimberArmPosition);
            motorTopRight.setPower(0.45);
        }
        else if (!motorTopRight.isBusy())
        {
            stateCode = endState;
            lastStateTimeStamp = System.currentTimeMillis();
            telemetry.addData("STATE", ": dump climber done");
        }

        return stateCode;
    }

    int touchButton(int startState, int endState) {
        int stateCode = startState;

        int armPosition = motorTopRight.getCurrentPosition();

        if ( Math.abs(armPosition-rightArmUpperLimit) > 400) {
            if (motorTopRight.getMode() != DcMotorController.RunMode.RUN_TO_POSITION) {
                motorTopRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            }
            motorTopRight.setTargetPosition(rightArmUpperLimit + 300);
            motorTopRight.setPower(0.3);

        }
        else if (!motorTopRight.isBusy())
        {
            stateCode = endState;
            motorTopRight.setPower(0.0);
            motorTopRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            lastStateTimeStamp = System.currentTimeMillis();
            telemetry.addData("STATE", ": move back arm done");
        }

        return stateCode;
    }
}