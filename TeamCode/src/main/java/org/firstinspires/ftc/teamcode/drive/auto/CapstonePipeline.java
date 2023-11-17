package org.firstinspires.ftc.teamcode.drive.auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CapstonePipeline extends OpenCvPipeline {

    /*
    Confidence of a position?
     */
    public enum CapstonePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public int color_index = 0;

    final Scalar RED = new Scalar(255, 0, 0);
    final Scalar GREEN = new Scalar(0, 255, 0);
    final Scalar BLUE = new Scalar(0, 0, 255);

    // TODO: Check Directions
    final Point LEFT_TOPLEFT_ANCHOR_POINT = new Point(0, 0);

    final int REGION_WIDTH = 1280 / 3;
    final int REGION_HEIGHT = 720;

    final Point CENTER_TOPLEFT_ANCHOR_POINT = new Point(LEFT_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, 0);
    final Point RIGHT_TOPLEFT_ANCHOR_POINT = new Point(CENTER_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, 0);

    final Point LEFT_BOTTOMRIGHT_ANCHOR_POINT = new Point(
            LEFT_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            LEFT_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    );

    final Point CENTER_BOTTOMRIGHT_ANCHOR_POINT = new Point(
            CENTER_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            CENTER_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    );

    final Point RIGHT_BOTTOMRIGHT_ANCHOR_POINT = new Point(
            RIGHT_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            RIGHT_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    );

    Mat leftRegionCb;
    int leftAvg;

    Mat centerRegionCb;
    int centerAvg;

    Mat rightRegionCb;
    int rightAvg;

    public volatile CapstonePosition position = CapstonePosition.LEFT;

//    /*
//     * This function takes the RGB frame, stores each color channel as an array in <side>Cr
//     */
//    void inputToCb(Mat input) {
//        // TODO: Invert channel
//        Imgproc.cvtColor(input, leftYCrCb, Imgproc.COLOR_RGB2BGR);
//        Core.extractChannel(leftYCrCb, leftCr, 1);
//
//        Imgproc.cvtColor(input, centerYCrCb, Imgproc.COLOR_RGB2BGR);
//        Core.extractChannel(centerYCrCb, centerCr, 1);
//
//        Imgproc.cvtColor(input, rightYCrCb, Imgproc.COLOR_RGB2BGR);
//        Core.extractChannel(rightYCrCb, rightCr, 1);
//    }

    @Override
    public void init(Mat firstFrame) {
        leftRegionCb = firstFrame.submat(new Rect(LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT));
        centerRegionCb = firstFrame.submat(new Rect(CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT));
        rightRegionCb = firstFrame.submat(new Rect(RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT));
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat target_channel = new Mat();
        Core.extractChannel(input, target_channel, color_index);

        leftRegionCb = input.submat(new Rect(LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT));
        centerRegionCb = input.submat(new Rect(CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT));
        rightRegionCb = input.submat(new Rect(RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT));

        leftAvg = (int) Core.mean(leftRegionCb).val[0];
        centerAvg = (int) Core.mean(centerRegionCb).val[0];
        rightAvg = (int) Core.mean(rightRegionCb).val[0];

        if (color_index == -1) {
            // find the avg farthest from the other two
            int total_avg = (leftAvg + centerAvg + rightAvg) / 3;
            int left_diff = Math.abs(total_avg - leftAvg);
            int center_diff = Math.abs(total_avg - centerAvg);
            int right_diff = Math.abs(total_avg - rightAvg);

            int max_diff = Math.max(left_diff, Math.max(center_diff, right_diff));

            if (max_diff == left_diff) {
                position = CapstonePosition.LEFT;
            } else if (max_diff == center_diff) {
                position = CapstonePosition.CENTER;
            } else {
                position = CapstonePosition.RIGHT;
            }
        } else {
            position = CapstonePosition.LEFT;

            if (centerAvg > leftAvg && centerAvg > rightAvg) {
                position = CapstonePosition.CENTER;
            } else if (rightAvg > leftAvg && rightAvg > centerAvg) {
                position = CapstonePosition.RIGHT;
            }
        }

        Imgproc.rectangle(
                input,
                LEFT_TOPLEFT_ANCHOR_POINT,
                LEFT_BOTTOMRIGHT_ANCHOR_POINT,
                RED,
                2
        );

        Imgproc.rectangle(
                input,
                CENTER_TOPLEFT_ANCHOR_POINT,
                CENTER_BOTTOMRIGHT_ANCHOR_POINT,
                GREEN,
                2
        );

        Imgproc.rectangle(
                input,
                RIGHT_TOPLEFT_ANCHOR_POINT,
                RIGHT_BOTTOMRIGHT_ANCHOR_POINT,
                BLUE,
                2
        );

        return input;
    }

    public int[] getAnalysis() {
        return new int[]{leftAvg, centerAvg, rightAvg};
    }

}
