package org.firstinspires.ftc.teamcode.drive.auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

public class CapstonePipeline extends OpenCvPipeline {

    /*
    Confidence of a position?
     */
    public enum CapstonePosition {
        LEFT, CENTER, RIGHT
    }

    public enum ColorMode {
        RED, GREEN, BLUE, ANY
    }

    public volatile ColorMode cmode = ColorMode.ANY;

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
    double leftAvg;

    Mat centerRegionCb;
    double centerAvg;

    Mat rightRegionCb;
    double rightAvg;

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

    public double maxDiff(List<Double> regions) {
        double avg = 0;
        for (Double sect : regions) {
            avg += sect;
        }

        avg /= regions.size();

        double maxDiff = 0;
        for (Double sect : regions) {
            double diff = Math.abs(avg - sect);

            if (diff > maxDiff) {
                maxDiff = diff;
            }
        }

        return maxDiff;
    }

    @Override
    public Mat processFrame(Mat input) {
        int color_index;
        switch (cmode) {
            case RED:
                color_index = 0;
                break;
            case GREEN:
                color_index = 1;
                break;
            case BLUE:
                color_index = 2;
                break;
            default:
                color_index = -1;
        }
        if (cmode != ColorMode.ANY) {
            Core.extractChannel(input, input, color_index);
        }

        leftRegionCb = input.submat(new Rect(LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT));
        centerRegionCb = input.submat(new Rect(CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT));
        rightRegionCb = input.submat(new Rect(RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT));

        Scalar leftMean = Core.mean(leftRegionCb);
        Scalar centerMean = Core.mean(centerRegionCb);
        Scalar rightMean = Core.mean(rightRegionCb);

        leftAvg = leftMean.val[0];
        centerAvg = centerMean.val[0];
        rightAvg = rightMean.val[0];

        if (cmode == ColorMode.ANY) { // TODO: Make this work with all channels
            // find the avg farthest from the other two
            double total_avg = (leftAvg + centerAvg + rightAvg) / 3.0;
            double left_diff = Math.abs(total_avg - leftAvg);
            double center_diff = Math.abs(total_avg - centerAvg);

            int max_diff = (int) maxDiff(Arrays.asList(leftAvg, centerAvg, rightAvg));

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

        // draw visual bounds on the image
        Imgproc.rectangle( // LEFT
                input,
                LEFT_TOPLEFT_ANCHOR_POINT,
                LEFT_BOTTOMRIGHT_ANCHOR_POINT,
                RED,
                position == CapstonePosition.LEFT ? 10 : 2
        );
        Imgproc.rectangle( // CENTER
                input,
                CENTER_TOPLEFT_ANCHOR_POINT,
                CENTER_BOTTOMRIGHT_ANCHOR_POINT,
                GREEN,
                position == CapstonePosition.CENTER ? 10 : 2
        );
        Imgproc.rectangle( // RIGHT
                input,
                RIGHT_TOPLEFT_ANCHOR_POINT,
                RIGHT_BOTTOMRIGHT_ANCHOR_POINT,
                BLUE,
                position == CapstonePosition.RIGHT ? 10 : 2
        );

        return input;
    }

    public double[] getAnalysis() {
        return new double[]{leftAvg, centerAvg, rightAvg};
    }

}
