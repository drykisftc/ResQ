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
    long crutchStartTime = 6000;
    int crutchStartDistance = 1000;

    @Override
    public void init() {
        super.init();
        teamColor = 'b';
    }

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
                    stateDryIce = goStraight(0,1,cruisePower, startLineToRampDistance, startLinetoRampTime);
                    break;
                case 1:
                    // turn
                    stateDryIce = turn(1, 2, turnPower, currentGyro, targetAngle);
                    if (stateDryIce == 2) {
                        turnStartTime = System.currentTimeMillis();
                        turnStartDistance = Math.min(leftWheelCurrent-leftWheelStartPos, rightWheelCurrent-rightWheelStartPos);
                    }
                    break;
                case 2:
                    stateDryIce = climbRamp();
                    break;
                case 3:
                    stateDryIce = holdPosition();
                    break;
                default:
                    stop();
                    // error
            }
        }
    }

    int climbRamp() {
        int retCode = 2;
        maintainAngleByEncoder(targetAngle,currentGyro,climbSpeed,climbPower);
        if (System.currentTimeMillis() - turnStartTime > crutchStartTime
                || getOdometer() - turnStartDistance > crutchStartDistance)
        {
            // deploy crutch
        }
        //retCode = 3;
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

