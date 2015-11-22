package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Canvas;
import android.graphics.Rect;
import android.hardware.Camera;
import android.hardware.Camera.Size;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import com.qualcomm.ftcrobotcontroller.CameraPreview;
import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;

import java.io.IOException;

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

    public Camera.Parameters cameraPara;

    private SurfaceHolder mHolder;

    public CameraPreview mPreview;

    ResQCamera () {
        if (FtcRobotControllerActivity.mCamera == null) {
            FtcRobotControllerActivity.bCameraOn = true;
            FtcRobotControllerActivity.mCamera = FtcRobotControllerActivity.openFrontFacingCamera();
            FtcRobotControllerActivity.mCamera.setDisplayOrientation(90);
            cameraPara = FtcRobotControllerActivity.mCamera.getParameters();
        }
    }

    public void turnOff() {
        FtcRobotControllerActivity.bCameraOn =false;
        FtcRobotControllerActivity.mCamera.stopPreview();
        FtcRobotControllerActivity.mCamera.release();
        FtcRobotControllerActivity.mCamera=null;
    }

    private Camera.PictureCallback mPicture = new Camera.PictureCallback() {

        @Override
        public void onPictureTaken(byte[] data, Camera camera) {
            Size size = camera.getParameters().getPictureSize();

            // judge whether the beacon is in sight
            if (data[0] > 0){
                fSkew = 0.0f;
            }

            // judge left/right beacon color

            // compute skew
        }
    };

    public void snapPicture () {
        if (FtcRobotControllerActivity.mCamera != null) {
            try {
                FtcRobotControllerActivity.mCamera.setPreviewDisplay(mHolder);
                FtcRobotControllerActivity.mCamera.startPreview();
            }catch (IOException e) {

            }
            //FtcRobotControllerActivity.mCamera.autoFocus(null);
            FtcRobotControllerActivity.mCamera.takePicture(null, mPicture, null);
            FtcRobotControllerActivity.mCamera.stopPreview();
        }
    }

    private boolean isBeaconInSight () {
        return false;
    }
}
