package org.firstinspires.ftc.teamcode.drive.utils;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

public class Vision {

    public AprilTagDetector AprilTagDetector;

    public VisionPortal visionPortal;
    public TFDetect tensorflowdetection;

    public Vision(HardwareMap hwMap) {
        AprilTagDetector = new AprilTagDetector();
        tensorflowdetection =  new TFDetect();
        tensorflowdetection.initTfod();

        try {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640, 480))
                    .enableLiveView(true)
                    .addProcessors(AprilTagDetector.aprilTag, tensorflowdetection.tfod)
                    .build();
        }
        catch (Exception ignored) {
        }
    }
    public void setExposure(){
        while (visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING) {
            Thread.yield();
        }


        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(15, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(255);
    }
}