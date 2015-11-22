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

import android.hardware.Camera;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Auto Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ResQTest extends OpMode {

    ColorSensor sensorRGB;

    OpticalDistanceSensor sensorODSLeft;
    OpticalDistanceSensor sensorODSRight;

    GyroSensor sensorGyro;

    ResQCamera camera;

    int refGyro = 0;
    int prevGyro = 0;
    int currentGyro = 0;

    GyroData gyroData;

    int minColorBrightness = 4;
    RGB rgb = new RGB(0,0,0);
    boolean enableLED = true;

    /**
     * Constructor
     */
    public ResQTest() {

    }

    private Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera)
        {
            Camera.Parameters parameters = camera.getParameters();

        }
    };

    /*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
    @Override
    public void init() {

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

        FtcRobotControllerActivity.mCamera.setPreviewCallback(previewCallback);

        ((FtcRobotControllerActivity)hardwareMap.appContext)
                .initPreview(FtcRobotControllerActivity.mCamera, camera, previewCallback);

        // wait 1 second for gyro calibration
        try {
            Thread.sleep(1500);                 //1000 milliseconds is one second.
        } catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        ResQUtils.getGyroData(sensorGyro, gyroData);
        currentGyro = gyroData.heading;
        refGyro = currentGyro;
        prevGyro = currentGyro;
        telemetry.addData("GYRO", "GYRO: "
                + " (" + String.format("%03d", currentGyro)
                + " ," + String.format("%03d", gyroData.xRotation)
                + " ," + String.format("%03d", gyroData.yRotation)
                + " ," + String.format("%03d", gyroData.zRotation) + ")");

    }

    public void start() {

        //currentGyro = getGyroHeading();
        //upateRotationData();
        ResQUtils.getGyroData(sensorGyro, gyroData);
        currentGyro = gyroData.heading;
        refGyro = currentGyro;
        prevGyro = currentGyro;
    }

    /*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
    @Override
    public void loop() {

        prevGyro = currentGyro;
        //currentGyro = getGyroHeading();
        //upateRotationData();
        ResQUtils.getGyroData(sensorGyro, gyroData);
        currentGyro = gyroData.heading;
        telemetry.addData("GYRO", " (" + String.format("%03d", currentGyro)
                + " ," + String.format("%03d", gyroData.xRotation)
                + " ," + String.format("%03d", gyroData.yRotation)
                + " ," + String.format("%03d", gyroData.zRotation) + ")");

        char color = ResQUtils.getColor(sensorRGB,1.3f,minColorBrightness,rgb);
        telemetry.addData("COLOR", ": " + color +
                " r=" + String.format("%d", rgb.r) +
                " g=" + String.format("%d", rgb.g) +
                " b=" + String.format("%d", rgb.b));

        int distanceLeft = sensorODSLeft.getLightDetectedRaw();
        int distanceRight = sensorODSRight.getLightDetectedRaw();

        telemetry.addData("DISTANCE", " ("
                + String.format("%03d", distanceLeft) + ", "
                + String.format("%03d", distanceRight) + ") ");

        try {
            camera.snapPicture();
        } catch (Exception e){
            telemetry.addData("ERRORMSG", e.toString());
        }
        telemetry.addData("CAMERA", " (Beacon skew: " + String.format("%.2g", camera.fSkew)
                + ", Seen: " + String.format("%b", camera.bBeaconSeen)
                + ", left: " + camera.cLeftColor
                + ", right: " + camera.cRightColor+ ")");
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
}

