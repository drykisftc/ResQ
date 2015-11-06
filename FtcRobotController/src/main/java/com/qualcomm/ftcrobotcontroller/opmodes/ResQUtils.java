package com.qualcomm.ftcrobotcontroller.opmodes;

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
}
