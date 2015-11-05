package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cController;

/**
 * Created by hfu on 11/4/15.
 */
public class GyroReader {

    private final I2cDevice device;
    private boolean transaction_complete;
    private boolean buffer_read_complete;
    private byte[] device_data;

    public GyroReader(I2cDevice i2cDevice, int i2cAddress, int memAddress, int num_bytes) {
        this.device = i2cDevice;
        device_data = null;
        transaction_complete = false;
        buffer_read_complete = false;
        i2cDevice.enableI2cReadMode(i2cAddress, memAddress, num_bytes);
        i2cDevice.setI2cPortActionFlag();
        i2cDevice.writeI2cCacheToController();
        i2cDevice.registerForI2cPortReadyCallback(new I2cController.I2cPortReadyCallback() {
            public void portIsReady(int port) {
                GyroReader.this.portDone();
            }
        });
    }

    public boolean isDone() {
        return this.transaction_complete && buffer_read_complete;
    }

    private void portDone() {
        if (!transaction_complete && device.isI2cPortReady()) {
            transaction_complete = true;
            device.readI2cCacheFromController();
        } else if (transaction_complete) {
            device_data = this.device.getCopyOfReadBuffer();
            device.deregisterForPortReadyCallback();
            buffer_read_complete = true;
        }
    }

    public byte[] getReadBuffer() {
        return device_data;
    }
}

