package org.firstinspires.ftc.teamcode.drive.auto;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Collections;

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

    double getMedian(Mat hist) {
        // binapprox algorithm

        long n = hist.total();
        int[] histBuff = new int[(int) n];
        hist.get(0, 0, histBuff);
        // Compute the mean and standard deviation
        // int n = x.length;
        double sum = 0;
        // int i;
        for (int i = 0; i < n; i++) {
            sum += histBuff[i];
        }
        double mu = sum / n;

        sum = 0;
        for (int i = 0; i < n; i++) {
            sum += (histBuff[i] - mu) * (histBuff[i] - mu);
        }
        double sigma = Math.sqrt(sum / n);

        // Bin x across the interval [mu-sigma, mu+sigma]
        int bottomcount = 0;
        int[] bincounts = new int[1001];
        for (int i = 0; i < 1001; i++) {
            bincounts[i] = 0;
        }
        double scalefactor = 1000 / (2 * sigma);
        double leftend = mu - sigma;
        double rightend = mu + sigma;
        int bin;

        for (int i = 0; i < n; i++) {
            if (histBuff[i] < leftend) {
                bottomcount++;
            } else if (histBuff[i] < rightend) {
                bin = (int) ((histBuff[i] - leftend) * scalefactor);
                bincounts[bin]++;
            }
        }

        double median = 0;
        // If n is odd
        if ((n % 2) != 0) {
            // Find the bin that contains the median
            int k = (int) ((n + 1) / 2);
            int count = bottomcount;

            for (int i = 0; i < 1001; i++) {
                count += bincounts[i];

                if (count >= k) {
                    median = (i + 0.5) / scalefactor + leftend;
                }
            }
        }

        // If n is even
        else {
            // Find the bins that contains the medians
            int k = (int) (n / 2);
            int count = bottomcount;

            for (int i = 0; i < 1001; i++) {
                count += bincounts[i];

                if (count >= k) {
                    int j = i;
                    while (count == k) {
                        j++;
                        count += bincounts[j];
                    }
                    median = (i + j + 1) / (2 * scalefactor) + leftend;
                }
            }
        }
        return median;
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

            // remove the alpha channel
            Imgproc.cvtColor(input, input, Imgproc.COLOR_BGRA2RGB);

            // first convert the red and blue channels to 1d arrays of pixels
            Mat redPixels = extractChannel(input, 0);
            Mat bluePixels = extractChannel(input, 2);

            redPixels.reshape(1, 1).convertTo(redPixels, CvType.CV_64F);
            bluePixels.reshape(1, 1).convertTo(bluePixels, CvType.CV_64F);

            // sort the arrays
            Core.sort(redPixels, redPixels, Core.SORT_EVERY_ROW + Core.SORT_ASCENDING);
            Core.sort(bluePixels, bluePixels, Core.SORT_EVERY_ROW + Core.SORT_ASCENDING);

            // only use the top 25% of the pixels
            int redPixelsToUse = (int) (redPixels.cols() * 0.25);
            int bluePixelsToUse = (int) (bluePixels.cols() * 0.25);

            redPixels = redPixels.colRange(redPixels.cols() - redPixelsToUse, redPixels.cols());
            bluePixels = bluePixels.colRange(bluePixels.cols() - bluePixelsToUse, bluePixels.cols());

            // create a histogram of the red and blue pixels
            Mat redHist = new Mat();
            Mat blueHist = new Mat();

            Imgproc.calcHist(Collections.singletonList(redPixels), new MatOfInt(0), new Mat(), redHist, new MatOfInt(256), new MatOfFloat(0, 256));
            Imgproc.calcHist(Collections.singletonList(bluePixels), new MatOfInt(0), new Mat(), blueHist, new MatOfInt(256), new MatOfFloat(0, 256));

            // clear the red and blue pixels mats
            redPixels.release();
            bluePixels.release();
            redHist.release();
            blueHist.release();

            double redMedian = getMedian(redHist);
            double blueMedian = getMedian(blueHist);

            double maxMean = Math.max(redMedian, blueMedian);

            Mat targetChannel;

            if (maxMean == redMedian) {
                targetChannel = extractChannel(input, 0);
                bestChannel = ColorMode.RED;
            } else if (maxMean == blueMedian) {
                targetChannel = extractChannel(input, 2);
                bestChannel = ColorMode.BLUE;
            } else {
                targetChannel = extractChannel(input, 2);
                bestChannel = ColorMode.ANY;
            }
            // clear the input mat
            input.release();

            // clip the input pixels to values between 216 and 255
            Core.inRange(targetChannel, new Scalar(216), new Scalar(255), targetChannel);

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

            input = targetChannel;
        } else {
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
        switch (position) {
            case LEFT:
                Imgproc.rectangle( // LEFT
                        input, LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT, WHITE, 15);
                break;
            case CENTER:
                Imgproc.rectangle( // CENTER
                        input, CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT, WHITE, 15);
                break;
            case RIGHT:
                Imgproc.rectangle( // RIGHT
                        input, RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT, WHITE, 15);
                break;
        }

        return input;
    }

    public double[] getAnalysis() {
        return new double[]{leftAvg, centerAvg, rightAvg};
    }

}
