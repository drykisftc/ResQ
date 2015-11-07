package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cController;

/**
 * Created by hfu on 11/4/15. Not thread safe. Must read buffer once then delete this
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

/**
 //        I2C Registers
 //        0x00 Sensor Firmware Revision
 //        0x01 Manufacturer Code
 //        0x02 Sensor ID Code
 //        0x03 Command
 //        0x04/0x05 Heading Data (lsb:msb)
 //        0x06/0x07 Integrated Z Value (lsb:msb)
 //        0x08/0x09 Raw X Value (lsb:msb)
 //        0x0A/0x0B Raw Y Value (lsb:msb)
 //        0x0C/0x0D Raw Z Value (lsb:msb)
 //        0x0E/0x0F Z Axis Offset (lsb:msb)
 //        0x10/0x11 Z Axis Scaling Coefficient (lsb:msb)
 */
//    void upateRotationData() {
//        //The Integrating Gyro I2C address is 0x20
//        GyroReader headingReader = new GyroReader(sensorGyro, 0x20, 0x00, 18);
//        while(!headingReader.isDone())
//        {
//        }
//        byte[] data = headingReader.getReadBuffer();
//        heading = data[4] | (data[5]<<8);
//        xRotation = data[8] | (data[9]<<8);
//        yRotation = data[10] | (data[11]<<8);
//        zRotation = data[12] | (data[13]<<8);
//    }

//    int getGyroHeading() {
//        //The Integrating Gyro I2C address is 0x20, heading is 0x04, 2 bytes (lsb:msb)
//        GyroReader headingReader = new GyroReader(sensorGyro, 0x20, 0x04, 2);
//        while(!headingReader.isDone())
//        {
//        }
//        byte[] h = headingReader.getReadBuffer(); //0x04/0x05 Heading Data (lsb:msb)
//        return h[0] | (h[1]<<8);
//    }