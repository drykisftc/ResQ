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

import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Auto Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class DryIceRamp extends DryIceAuto {

    int startLineToRampDistance = 6000;
    long startLinetoRampTime = 5000; // 5 seconds
    int climbSpeed = 100;
    float climbPower = 1.0f;
    long turnStartTime =0;
    int turnStartDistance= 0;
    long crutchStartTime = 3000;
    long crutchStopTime = 8000;

    int crutchStartDistance = 1000;

    @Override
    public void init() {
        super.init();
        teamColor = 'b';
//        StarLineToCenterLineDistance = 2000;
//        CenterlineToBeaconLineDistance = 5399;
//        BeaconLineToBeaconDistance = 100;
        StarLineToCenterLineDistance = 2850;
        CenterlineToBeaconLineDistance = 8650; // 9500 before
        BeaconLineToBeaconDistance = 7500;
        RampLineToBeaconLineDistance = 3500;
        turnAngle = 60;
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
                    turnAngle = 20;
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

    int backup2(int startState, int endState) {
        if (teamColor == 'b') {
            telemetry.addData("STATE", ": Turning right...");
            targetAngle = normalizeAngle(targetAngle - 45);
        } else {
            telemetry.addData("STATE", ": Turning left...");
            targetAngle = normalizeAngle(targetAngle + 45);
        }
        return endState;
    }

    int findRamp2(int startState, int endState) {
        return turn(startState, endState, 0.0f, currentGyro, targetAngle);
    }

    int climbRamp2(int startState, int endState) {
        int retCode = startState;
        maintainAngleByEncoder(targetAngle,currentGyro,climbSpeed,climbPower);
        long elapse = System.currentTimeMillis() - turnStartTime;
        if ( elapse > crutchStartTime
                || getOdometer() - turnStartDistance > crutchStartDistance)
        {
            // relax right arm
            motorTopRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorTopRight.setPower(0.0);

            // deploy crutch
            if (elapse < crutchStopTime) {
                elevator.setPosition(elevatorUpPosition);
            }
            else {
                elevator.setPosition(elevatorStopPosition);
            }
        }

        if ( elapse > timeBudget) {
            retCode = endState;
        }
        return retCode;
    }

    int holdPosition() {
        motorBottomLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBottomLeft.setTargetPosition(motorBottomLeft.getCurrentPosition());
        motorBottomLeft.setPower(0.05);
        motorBottomRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBottomRight.setTargetPosition(motorBottomRight.getCurrentPosition());
        motorBottomRight.setPower(0.05);
        return 3;
    }
}

