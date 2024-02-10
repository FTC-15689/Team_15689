package org.firstinspires.ftc.teamcode.drive.auto;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Collections;
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
        int median = hist.rows() / 2;

        if (hist.rows() % 2 == 1) {
            return hist.get(median, 0)[0];
        } else {
            return (hist.get(median - 1, 0)[0] + hist.get(median, 0)[0]) / 2;
        }
    }

    private Mat extractChannel(Mat src, int index) {
        Mat tmp = new Mat();
        Core.extractChannel(src, tmp, index);

        return tmp;
    }

    @Override
    public Mat processFrame(Mat input) {
        // extract the red and blue channels
        Mat redChannel = extractChannel(input, 0);
        Mat blueChannel = extractChannel(input, 2);

        // find which channel has the higher std dev
        MatOfDouble redMean = new MatOfDouble();
        MatOfDouble redStd = new MatOfDouble();
        Core.meanStdDev(redChannel, redMean, redStd);

        MatOfDouble blueMean = new MatOfDouble();
        MatOfDouble blueStd = new MatOfDouble();
        Core.meanStdDev(blueChannel, blueMean, blueStd);

        double redVal = redStd.get(0, 0)[0];
        double blueVal = blueStd.get(0, 0)[0];

        Mat targetChannel;
        Mat avg_others;

        if (redVal > blueVal) {
            bestChannel = ColorMode.RED;
            blueChannel.release();
            targetChannel = redChannel.clone();
            redChannel.release();

            avg_others = new Mat();
            Core.extractChannel(input, avg_others, 1);
            Core.add(avg_others, extractChannel(input, 2), avg_others);
            Core.divide(avg_others, new Scalar(2), avg_others);
        } else {
            bestChannel = ColorMode.BLUE;
            redChannel.release();
            targetChannel = blueChannel.clone();
            blueChannel.release();

            avg_others = new Mat();
            Core.extractChannel(input, avg_others, 0);
            Core.add(avg_others, extractChannel(input, 1), avg_others);
            Core.divide(avg_others, new Scalar(2), avg_others);
        }

        // subtract the average of the other two channels from the target channel thus removing the background
        Core.subtract(targetChannel, avg_others, targetChannel);
        avg_others.release();
        Core.inRange(targetChannel, new Scalar(0), new Scalar(255), targetChannel);

        // pass a threshold of 100 to the target channel
        Imgproc.threshold(targetChannel, targetChannel, 50, 255, Imgproc.THRESH_BINARY);

        // find the edges in the target channel
        Mat edges = new Mat();
        Imgproc.Canny(targetChannel, edges, 100, 200);

        // dilate the edges
        Imgproc.dilate(edges, edges, new Mat(), new Point(-1, -1), 1);

        // find the contours in the edges
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new java.util.ArrayList<>();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // sort the contours by area
        contours.sort((o1, o2) -> {
            double area1 = Imgproc.contourArea(o1);
            double area2 = Imgproc.contourArea(o2);
            return Double.compare(area2, area1);
        });

        // only use the first five if there are more than five
        if (contours.size() > 5) {
            contours = contours.subList(0, 5);
        }

        // draw the contours on the input
        input.convertTo(input, CvType.CV_8U);
        Imgproc.drawContours(input, contours, -1, bestChannel == ColorMode.RED ? RED : BLUE, 15);

        // Using the center of the bounding box around each contour, group them into the left, center, and right regions
        List<List<MatOfPoint>> leftRegions = new java.util.ArrayList<>();
        List<List<MatOfPoint>> centerRegions = new java.util.ArrayList<>();
        List<List<MatOfPoint>> rightRegions = new java.util.ArrayList<>();

        int leftX = (int) (input.width() / 3.0);
        int rightX = (int) (input.width() * 2.0 / 3.0);
        int centerX = (int) (input.width() / 2.0);

        for (MatOfPoint contour : contours) {
            Rect boundingBox = Imgproc.boundingRect(contour);
            Point center = new Point(boundingBox.x + boundingBox.width / 2.0, boundingBox.y + boundingBox.height / 2.0);

            if (center.x < leftX) {
                leftRegions.add(Collections.singletonList(contour));
            } else if (center.x > rightX) {
                rightRegions.add(Collections.singletonList(contour));
            } else {
                centerRegions.add(Collections.singletonList(contour));
            }
        }

        // find the group with the most area
        double leftArea = leftRegions.stream().mapToDouble(region -> Imgproc.contourArea(region.get(0))).sum();
        double centerArea = centerRegions.stream().mapToDouble(region -> Imgproc.contourArea(region.get(0))).sum();
        double rightArea = rightRegions.stream().mapToDouble(region -> Imgproc.contourArea(region.get(0))).sum();

        if (centerArea > leftArea && centerArea > rightArea) {
            position = CapstonePosition.CENTER;

            // draw the center region
            Imgproc.rectangle(input, new Point(centerX - REGION_WIDTH / 2.0, 0), new Point(centerX + REGION_WIDTH / 2.0, input.height()), WHITE, 15);
        }
        else if (rightArea > leftArea && rightArea > centerArea) {
            position = CapstonePosition.RIGHT;

            // draw the right region
            Imgproc.rectangle(input, new Point(rightX - REGION_WIDTH / 2.0, 0), new Point(rightX + REGION_WIDTH / 2.0, input.height()), WHITE, 15);
        }
        else {
            position = CapstonePosition.LEFT;

            // draw the left region
            Imgproc.rectangle(input, new Point(leftX - REGION_WIDTH / 2.0, 0), new Point(leftX + REGION_WIDTH / 2.0, input.height()), WHITE, 15);
        }

        // convert input to int
        input.convertTo(input, CvType.CV_8U);
        targetChannel.convertTo(targetChannel, CvType.CV_8U);

        return input;
    }

    public double[] getAnalysis() {
        return new double[]{leftAvg, centerAvg, rightAvg};
    }

}
