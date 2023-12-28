package org.firstinspires.ftc.teamcode.drive.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class CapstoneDetectionCamera {
    final WebcamName webcam1;
    final OpenCvCamera camera;
    public final CapstonePipeline pipeline;
    Telemetry telemetry;

    public CapstoneDetectionCamera(HardwareMap hardwareMap, int color_index) {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        @SuppressLint("DiscouragedApi") int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1);

        pipeline = new CapstonePipeline();
        switch (color_index) {
            case 0:
                pipeline.cmode = CapstonePipeline.ColorMode.RED;
                break;
            case 1:
                pipeline.cmode = CapstonePipeline.ColorMode.GREEN;
                break;
            case 2:
                pipeline.cmode = CapstonePipeline.ColorMode.BLUE;
                break;
            default:
                pipeline.cmode = CapstonePipeline.ColorMode.ANY;
        }
        camera.setPipeline(pipeline);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                                             FtcDashboard.getInstance().startCameraStream(camera, 0);
                                         }

                                         @Override
                                         public void onError(int errorCode) {
                                             System.out.println("error" + errorCode);
                                             telemetry.addData("Error has occurred. Error code - ", errorCode);
                                         }
                                     }
        );
    }

    public CapstonePipeline.CapstonePosition getPosition() {
        return pipeline.position;
    }

    public double[] getAnalysis() {
        return pipeline.getAnalysis();
    }
}