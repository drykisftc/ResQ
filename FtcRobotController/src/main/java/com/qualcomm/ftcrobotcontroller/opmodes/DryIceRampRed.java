package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by hfu on 11/2/15.
 */
public class DryIceRampRed extends DryIceRamp {
    @Override
    public void init() {
        super.init();
        teamColor = 'r';
    }
}
