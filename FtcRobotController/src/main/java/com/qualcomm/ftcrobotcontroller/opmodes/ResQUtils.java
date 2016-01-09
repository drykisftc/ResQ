package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by hfu on 11/3/15.
 */

public class ResQUtils {

    static float lookUpTableFunc (float dVal, float[] scaleArray ) {
        int size = scaleArray.length-1;
        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * size);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > size) {
            index = size;
        }

        // get value from the array.
        float dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        // return scaled value.
        return dScale;
    }

    static void getGyroData (GyroSensor gyro, GyroData data){
        data.heading =gyro.getHeading();
        data.xRotation = gyro.rawX();
        data.yRotation = gyro.rawY();
        data.zRotation = gyro.rawZ();
    }

    static char getColor(ColorSensor cs,
                         float snrLimit,
                         int minBrightness,
                         RGB rgb) {
        rgb.r = cs.red();
        rgb.g = cs.green();
        rgb.b = cs.blue();

        // find the max
        int m = Math.max(rgb.r, rgb.g);
        m = Math.max(m, rgb.b);
        int sum = rgb.r + rgb.g + rgb.b;

        // if SNR is good
        if (sum > minBrightness && m > sum * 0.333 * snrLimit) {
            if (m == rgb.g) return 'g';
            if (m == rgb.r) return 'r';
            if (m == rgb.b) return 'b';
        }
        return 'u'; // unknown color
    }

    static int followColorLine(char colorFollowed,
                               ColorSensor cs,
                               float snrLimit,
                               int minBrightness,
                               DcMotor left,
                               DcMotor right,
                               float powerForward,
                               float powerTurn) {
        RGB rgb = new RGB(0,0,0);
        char color = getColor(cs, snrLimit, minBrightness, rgb);

        int intensity = rgb.getIntensity();

        if ( color != colorFollowed || intensity < minBrightness)
        {
            left.setPower(Range.clip(powerForward+powerTurn,-1,1));
            right.setPower(Range.clip(powerForward - powerTurn, -1, 1));
        }
        else {
            left.setPower(Range.clip(powerForward-powerTurn,-1,1));
            right.setPower(Range.clip(powerForward+powerTurn,-1,1));
        }

        return 0;
    }

    static int moveMotorByEncoder (DcMotor motor, int pos, int tolerance, double maxDelta, float [] lut ) {

        if (motor.getMode() != DcMotorController.RunMode.RUN_USING_ENCODERS) {
            motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        int currentPos = motor.getCurrentPosition();
        int deltaPos = pos - currentPos;

        if (Math.abs(deltaPos) > tolerance) {
            // map deltaPos to power
            double delta = Range.clip(deltaPos/maxDelta, -1.0, 1.0);

            // set power
            motor.setPower(ResQUtils.lookUpTableFunc((float) delta, lut));
        }
        else {
            motor.setPower(0);
        }
        return  currentPos;
    }

    static void moveMotorByPower(DcMotor motor, float power) {
        if (motor.getMode() != DcMotorController.RunMode.RUN_USING_ENCODERS) {
            motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        motor.setPower(power);
    }

    static void holdMotorByEncoder(DcMotor motor, int pos, float power){
        if (!motor.isBusy()) {
            motor.setTargetPosition(pos);
            if(motor.getMode() != DcMotorController.RunMode.RUN_TO_POSITION) {
                motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            }
            motor.setPower(power);
        }
    }
}
