package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by hfu on 11/2/15.
 */
public class DryIceAutoRed extends DryIceAuto {
    @Override
    public void init() {
        teamColor = 'r';
        super.init();
    }
}
