package org.firstinspires.ftc.teamcode.drive.auto;

import org.opencv.core.Core;
import org.opencv.core.CvType;
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
    final Scalar WHITE = new Scalar(255, 255, 255);

    final int REGION_WIDTH = 1280 / 3;
    final int REGION_HEIGHT = 720;
    final Point LEFT_TOPLEFT_ANCHOR_POINT = new Point(0, 0);

    final Point CENTER_TOPLEFT_ANCHOR_POINT = new Point(LEFT_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, 0);
    final Point RIGHT_TOPLEFT_ANCHOR_POINT = new Point(CENTER_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, 0);

    final Point LEFT_BOTTOMRIGHT_ANCHOR_POINT = new Point(LEFT_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, LEFT_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    final Point CENTER_BOTTOMRIGHT_ANCHOR_POINT = new Point(CENTER_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, CENTER_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    final Point RIGHT_BOTTOMRIGHT_ANCHOR_POINT = new Point(RIGHT_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, RIGHT_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat leftRegionCb;
    double leftAvg;

    Mat centerRegionCb;
    double centerAvg;

    Mat rightRegionCb;
    double rightAvg;

    public volatile CapstonePosition position = CapstonePosition.LEFT;
    public volatile ColorMode bestChannel = ColorMode.ANY;

    @Override
    public void init(Mat firstFrame) {
        leftRegionCb = firstFrame.submat(new Rect(LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT));
        centerRegionCb = firstFrame.submat(new Rect(CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT));
        rightRegionCb = firstFrame.submat(new Rect(RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT));
    }

    public double maxDiff(List<Double> regions) {
        double total_avg = 0;
        for (Double sect : regions) {
            total_avg += sect;
        }

        total_avg /= regions.size();

        double maxDiff = 0;
        for (Double region : regions) {
            double diff = Math.abs(total_avg - region);

            if (diff > maxDiff) {
                maxDiff = diff;
            }
        }

        return maxDiff;
    }

    public double calcMeans(Mat input) {
        leftRegionCb = input.submat(new Rect(LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT));
        centerRegionCb = input.submat(new Rect(CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT));
        rightRegionCb = input.submat(new Rect(RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT));

        Scalar leftMean = Core.mean(leftRegionCb);
        Scalar centerMean = Core.mean(centerRegionCb);
        Scalar rightMean = Core.mean(rightRegionCb);

        leftAvg = leftMean.val[0];
        centerAvg = centerMean.val[0];
        rightAvg = rightMean.val[0];

        return Math.max(leftAvg, Math.max(centerAvg, rightAvg));
    }

    private Mat extractChannel(Mat src, int index) {
        Mat tmp = new Mat();
        Core.extractChannel(src, tmp, index);

        return tmp;
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

        if (cmode == ColorMode.ANY) {
            // convert the bgr
            Imgproc.cvtColor(input, input, Imgproc.COLOR_BGRA2BGR);

            // create a luminosity mask and clip values that are relatively dark
            Mat luminosity = extractChannel(input, 0);
            CvType.
            Core.add(luminosity, extractChannel(input, 2), luminosity);

            Core.multiply(luminosity, Mat.ones(luminosity.size(), CvType.CV_8U), luminosity, 0.5);

            luminosity = Core

            Imgproc.adaptiveThreshold(luminosity, luminosity, 1, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY_INV, 15, 40);

            Core.merge(Arrays.asList(luminosity, luminosity, luminosity), luminosity);
            // multiply our luminosity mask against the image
            Core.multiply(input, luminosity, input);

            // create a list of the channels and an empty list to store the differences of each channel
            double redMean = calcMeans(extractChannel(input, 0));
            double blueMean = calcMeans(extractChannel(input, 2));

            double maxMean = Math.max(redMean, blueMean);

            Mat targetChannel;

            if (maxMean == redMean) {
                targetChannel = extractChannel(input, 0);
                bestChannel = ColorMode.RED;
            } else if (maxMean == blueMean) {
                targetChannel = extractChannel(input, 2);
                bestChannel = ColorMode.BLUE;
            } else {
                targetChannel = extractChannel(input, 2);
                bestChannel = ColorMode.ANY;
            }

            leftRegionCb = targetChannel.submat(new Rect(LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT));
            centerRegionCb = targetChannel.submat(new Rect(CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT));
            rightRegionCb = targetChannel.submat(new Rect(RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT));

            Scalar leftMean = Core.mean(leftRegionCb);
            Scalar centerMean = Core.mean(centerRegionCb);
            Scalar rightMean = Core.mean(rightRegionCb);

            leftAvg = leftMean.val[0];
            centerAvg = centerMean.val[0];
            rightAvg = rightMean.val[0];

            maxMean = Math.max(leftAvg, Math.max(centerAvg, rightAvg));

            if (maxMean == leftAvg) {
                position = CapstonePosition.LEFT;
            } else if (maxMean == centerAvg) {
                position = CapstonePosition.CENTER;
            } else {
                position = CapstonePosition.RIGHT;
            }
        }
        else {
            leftRegionCb = input.submat(new Rect(LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT));
            centerRegionCb = input.submat(new Rect(CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT));
            rightRegionCb = input.submat(new Rect(RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT));

            Scalar leftMean = Core.mean(leftRegionCb);
            Scalar centerMean = Core.mean(centerRegionCb);
            Scalar rightMean = Core.mean(rightRegionCb);

            leftAvg = leftMean.val[0];
            centerAvg = centerMean.val[0];
            rightAvg = rightMean.val[0];

            position = CapstonePosition.LEFT;

            if (centerAvg > leftAvg && centerAvg > rightAvg) {
                position = CapstonePosition.CENTER;
            } else if (rightAvg > leftAvg && rightAvg > centerAvg) {
                position = CapstonePosition.RIGHT;
            }
        }

        // draw visual bounds on the image
        Imgproc.rectangle( // LEFT
                input, LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT, position == CapstonePosition.LEFT ? WHITE : BLUE, 5);
        Imgproc.rectangle( // CENTER
                input, CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT, position == CapstonePosition.CENTER ? WHITE : BLUE, 5);
        Imgproc.rectangle( // RIGHT
                input, RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT, position == CapstonePosition.RIGHT ? WHITE : BLUE, 5);

        return input;
    }

    public double[] getAnalysis() {
        return new double[]{leftAvg, centerAvg, rightAvg};
    }

}
