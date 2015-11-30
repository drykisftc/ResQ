package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by hfu on 11/28/15.
 */
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
