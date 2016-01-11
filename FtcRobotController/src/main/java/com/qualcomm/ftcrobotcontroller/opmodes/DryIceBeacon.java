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

    @Override
    public void init() {
        super.init();
        teamColor = 'b';
        StarLineToCenterLineDistance = 6000;
        CenterlineToBeaconLineDistance = 8000;
        BeaconLineToBeaconDistance = 2100;
        RampLineToBeaconLineDistance = 3500;
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
                    stateDryIce = turn(3, 4, 0.0f, currentGyro, targetAngle);
                    break;
                case 4:
                    stateDryIce = goStraightFromBeaconlineToBeacon(4, 5, searchPower, 25000);
                    // find the beacon
                    //stateDryIce = searchBeacon(searchPower, 4, 5);
                    break;
                case 5:
                    // touch the button
                    stateDryIce = touchButton(5, 6);
                    break;
                case 6:
                    // backup
                    stateDryIce = backup(6, 7);
                    break;
                case 7:
                    // find the ramp with correct color
                    stateDryIce = findRamp(7, 8);
                    break;
                case 8:
                    // climb up the ramp
                    stateDryIce = climbRamp(8, 9);
                    break;
                default:
                    stop();
                    // error

            }
        }
    }
}