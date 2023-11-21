package org.firstinspires.ftc.teamcode.drive.auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Dictionary;
import java.util.Hashtable;
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

    // TODO: Check Directions
    final Point LEFT_TOPLEFT_ANCHOR_POINT = new Point(0, 0);

    final int REGION_WIDTH = 1280 / 3;
    final int REGION_HEIGHT = 720;

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

        if (cmode == ColorMode.ANY) { // TODO: Make this work with all channels
            // create a list of the channels and an empty list to store the differences of each channel
            List<Mat> channels = new ArrayList<>();
            Core.split(input, channels);
            List<Dictionary<Integer, Double>> diffs = new ArrayList<>(channels.size());

            // iterate over channels and find the difference between the average of the channel and the average of the other channels
            for (Mat channel : channels) {
                leftRegionCb = channel.submat(new Rect(LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT));
                centerRegionCb = channel.submat(new Rect(CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT));
                rightRegionCb = channel.submat(new Rect(RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT));

                Scalar leftMean = Core.mean(leftRegionCb);
                Scalar centerMean = Core.mean(centerRegionCb);
                Scalar rightMean = Core.mean(rightRegionCb);

                leftAvg = leftMean.val[0];
                centerAvg = centerMean.val[0];
                rightAvg = rightMean.val[0];

                double total_avg = (leftAvg + centerAvg + rightAvg) / 3.0;
                double left_diff = Math.abs(total_avg - leftAvg);
                double center_diff = Math.abs(total_avg - centerAvg);
                double right_diff = Math.abs(total_avg - rightAvg);

                double max_diff = maxDiff(Arrays.asList(left_diff, center_diff, right_diff));

                if (max_diff == left_diff) {
                    diffs.add(new Hashtable<Integer, Double>() {{
                        put(0, left_diff);
                    }});
                } else if (max_diff == center_diff) {
                    diffs.add(new Hashtable<Integer, Double>() {{
                        put(1, center_diff);
                    }});
                } else {
                    diffs.add(new Hashtable<Integer, Double>() {{
                        put(2, right_diff);
                    }});
                }
            }

            // the diffs list now contains a list of dictionaries, each dictionary containing the difference of the channel at the index of the dictionary
            // iterate over the diffs list and find the channel with the biggest difference
            double max_diff = 0;
            int max_channel_index = 0;

            for (Dictionary<Integer, Double> diff : diffs) {
                // the dictionary is a single k, v pair
                int channel_index = diff.keys().nextElement();
                double channel_diff = diff.get(channel_index);

                if (channel_diff > max_diff) {
                    max_diff = channel_diff;
                    max_channel_index = channel_index;
                }
            }

            // set the cmode to the channel with the biggest difference
            switch ((int) max_channel_index) {
                case 1:
                    position = CapstonePosition.CENTER;
                    break;
                case 2:
                    position = CapstonePosition.RIGHT;
                    break;
                default:
                    position = CapstonePosition.LEFT;
            }

            // replace input with the channel that had the biggest difference
            Core.extractChannel(input, input, max_channel_index);
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
                input, LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT, position == CapstonePosition.LEFT ? GREEN : RED, 5);
        Imgproc.rectangle( // CENTER
                input, CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT, position == CapstonePosition.CENTER ? GREEN : BLUE, 5);
        Imgproc.rectangle( // RIGHT
                input, RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT, position == CapstonePosition.RIGHT ? GREEN : WHITE, 5);

        // draw the average values on the image
        Imgproc.putText(input, "Left: " + leftAvg, new Point(LEFT_TOPLEFT_ANCHOR_POINT.x, LEFT_TOPLEFT_ANCHOR_POINT.y - 10), 0, 1, position == CapstonePosition.LEFT ? GREEN : WHITE, 2);
        Imgproc.putText(input, "Center: " + centerAvg, new Point(CENTER_TOPLEFT_ANCHOR_POINT.x, CENTER_TOPLEFT_ANCHOR_POINT.y - 10), 0, 1, position == CapstonePosition.CENTER ? GREEN : WHITE, 2);
        Imgproc.putText(input, "Right: " + rightAvg, new Point(RIGHT_TOPLEFT_ANCHOR_POINT.x, RIGHT_TOPLEFT_ANCHOR_POINT.y - 10), 0, 1, position == CapstonePosition.RIGHT ? GREEN : WHITE, 2);

        return input;
    }

    public double[] getAnalysis() {
        return new double[]{leftAvg, centerAvg, rightAvg};
    }

}
