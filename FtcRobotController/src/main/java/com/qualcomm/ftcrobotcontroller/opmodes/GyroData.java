package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by hfu on 11/5/15.
 */
public class GyroData {
    int heading =0;
    int xRotation=0;
    int yRotation=0;
    int zRotation=0;
    public GyroData (int h, int x, int y, int z)
    {
        heading = h;
        xRotation = x;
        yRotation = y;
        zRotation = z;
    }
}
