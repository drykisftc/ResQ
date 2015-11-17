package com.qualcomm.ftcrobotcontroller.opmodes;

import android.hardware.Camera;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;

/**
 * Created by hfu on 11/16/15.
 */



//  public void initPreview(final Camera camera, final CameraOp context, final Camera.PreviewCallback previewCallback) {
//    runOnUiThread(new Runnable() {
//      @Override
//      public void run() {
//        context.preview = new CameraPreview(FtcRobotControllerActivity.this, camera, previewCallback);
//        FrameLayout previewLayout = (FrameLayout) findViewById(R.id.previewLayout);
//        previewLayout.addView(context.preview);
//      }
//    });
//  }
public class ResQCamera {

    public Boolean bBeaconSeen = false;

    public char cLeftColor = 'b';

    public char cRightColor = 'r';

    public float fSkew = 0.0f;

    ResQCamera () {
        if (FtcRobotControllerActivity.mCamera == null) {
            FtcRobotControllerActivity.bCameraOn = true;
            FtcRobotControllerActivity.mCamera = FtcRobotControllerActivity.openFrontFacingCamera();
        }
    }

    public void turnOff() {
        FtcRobotControllerActivity.bCameraOn =false;
        FtcRobotControllerActivity.mCamera.release();
    }

    private Camera.PictureCallback mPicture = new Camera.PictureCallback() {

        @Override
        public void onPictureTaken(byte[] data, Camera camera) {

            // judge whether the beacon is in sight

            // judge left/right beacon color

            // compute skew
        }
    };

    public void snapPicture () {
        if (FtcRobotControllerActivity.mCamera != null) {
            FtcRobotControllerActivity.mCamera.autoFocus(null);
            FtcRobotControllerActivity.mCamera.takePicture(null, null, mPicture);
        }
    }
}
