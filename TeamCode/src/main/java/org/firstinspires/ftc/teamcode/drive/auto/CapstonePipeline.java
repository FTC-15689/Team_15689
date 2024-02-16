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

import java.util.List;

public class CapstonePipeline extends OpenCvPipeline {

    private Mat edges;
    private Mat hierarchy;

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

    // Mat variables
    private Mat redChannel;
    private Mat blueChannel;
    private Mat rgChannel;
    private Mat bgChannel;
    private Mat targetChannel;
    private Mat prevTargetChannel;

    @Override
    public void init(Mat firstFrame) {
        leftRegionCb = firstFrame.submat(new Rect(LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT));
        centerRegionCb = firstFrame.submat(new Rect(CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT));
        rightRegionCb = firstFrame.submat(new Rect(RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT));
    }

    private Mat extractChannel(Mat src, int index) {
        Mat tmp = new Mat();
        Core.extractChannel(src, tmp, index);

        return tmp;
    }

    private Mat clip(Mat src, double min, double max) {
        Mat tmp = new Mat();
        Mat below = new Mat();
        Mat above = new Mat();
        src.copyTo(tmp);

        // create a mask of the pixels where below min -> 0 else 1
        Core.compare(tmp, new Scalar(min), below, Core.CMP_LT);
        // create a mask of the pixels where above max -> 0 else 1
        Core.compare(tmp, new Scalar(max), above, Core.CMP_GT);

        // set the pixels where below min to min
        tmp.setTo(new Scalar(min), below);
        // set the pixels where above max to max
        tmp.setTo(new Scalar(max), above);

        return tmp;
    }

    /**
     * Clip the values of src to CV_8U
     * @param src the mat to clip as a 1 channel mat
     * @return the clipped mat
     */
    private Mat clip(Mat src) {
        double min = 0;
        double max = Math.pow(2, 8) - 1;

        return clip(src, min, max);
    }

    @Override
    public Mat processFrame(Mat input) {
        // extract the red and blue channels
        redChannel = extractChannel(input, 0);
        redChannel.convertTo(redChannel, CvType.CV_16S);
        blueChannel = extractChannel(input, 2);
        blueChannel.convertTo(blueChannel, CvType.CV_16S);
        targetChannel = new Mat();

        // find the diff mats
        rgChannel = new Mat();
        bgChannel = new Mat();

        rgChannel = extractChannel(input, 1);
        rgChannel.convertTo(rgChannel, CvType.CV_16S);
        bgChannel = extractChannel(input, 1);
        bgChannel.convertTo(bgChannel, CvType.CV_16S);

        Core.add(redChannel, rgChannel, rgChannel);
        Core.add(blueChannel, bgChannel, bgChannel);

        // divide the rg and bg channels by 2.0
        Core.divide(rgChannel, new Scalar(2.0), rgChannel);
        Core.divide(bgChannel, new Scalar(2.0), bgChannel);

        // take the difference of r-bg and b-rg
        Core.subtract(redChannel, bgChannel, bgChannel);
        Core.subtract(blueChannel, rgChannel, rgChannel);

        // clip the negative values
        rgChannel = clip(rgChannel);
        bgChannel = clip(bgChannel);

        // find the std of the rg and bg channels
        MatOfDouble mean_rg = new MatOfDouble();
        MatOfDouble std_rg = new MatOfDouble();
        Core.meanStdDev(rgChannel, mean_rg, std_rg);

        MatOfDouble mean_bg = new MatOfDouble();
        MatOfDouble std_bg = new MatOfDouble();
        Core.meanStdDev(bgChannel, mean_bg, std_bg);

        double redVal = std_rg.get(0, 0)[0];
        double blueVal = std_bg.get(0, 0)[0];

        mean_rg.release();
        std_rg.release();
        mean_bg.release();
        std_bg.release();

        if (redVal > blueVal) {
            bestChannel = ColorMode.RED;
            redChannel.copyTo(targetChannel);
        } else {
            bestChannel = ColorMode.BLUE;
            blueChannel.copyTo(targetChannel);
        }
        bestChannel = ColorMode.RED;
        bgChannel.copyTo(targetChannel);

        redChannel.release();
        blueChannel.release();
        rgChannel.release();
        bgChannel.release();

        targetChannel.convertTo(targetChannel, CvType.CV_8U);

        if (prevTargetChannel == null) {
            prevTargetChannel = targetChannel;

            bestChannel = ColorMode.RED;
            position = CapstonePosition.LEFT;

            return targetChannel;
        }

        // sum the target and previous channel then clip to above 255
        targetChannel.convertTo(targetChannel, CvType.CV_16S);
        prevTargetChannel.convertTo(prevTargetChannel, CvType.CV_16S);

        Core.add(targetChannel, prevTargetChannel, targetChannel);
        Core.subtract(targetChannel, new Scalar(255), targetChannel);
        clip(targetChannel);

        // pass a threshold of 50 to the target channel
        Imgproc.threshold(targetChannel, targetChannel, 50, 255, Imgproc.THRESH_BINARY);

        // convert the target channel to 8U
        targetChannel.convertTo(targetChannel, CvType.CV_8U);

        // find the edges in the target channel
        edges = new Mat();
        Imgproc.Canny(targetChannel, edges, 100, 200);

        // dilate the edges
        Imgproc.dilate(edges, edges, new Mat(), new Point(-1, -1), 1);

        // find the contours in the edges
        hierarchy = new Mat();
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

        // Using the center of the bounding box around each contour, group them into the left and right regions
        double leftArea = 0;
        double rightArea = 0;

        for (MatOfPoint contour : contours) {
            Rect boundingBox = Imgproc.boundingRect(contour);
            Point center = new Point(boundingBox.x + boundingBox.width / 2.0, boundingBox.y + boundingBox.height / 2.0);
            if (center.x < (double) (REGION_WIDTH * 3) / 2) {
                leftArea += Imgproc.contourArea(contour);
            } else if (center.x > (double) (REGION_WIDTH * 3) / 2) {
                rightArea += Imgproc.contourArea(contour);
            }
        }

        // see if the two areas are similar and if so then the position is LEFT
        if (Math.abs(leftArea - rightArea) < 1000) {
            position = CapstonePosition.LEFT;
        } else if (leftArea > rightArea) {
            position = CapstonePosition.CENTER;
        } else {
            position = CapstonePosition.RIGHT;
        }

        // draw a rectangle on the input to show the regions
        switch (position) {
            case LEFT:
                Imgproc.rectangle(input, LEFT_TOPLEFT_ANCHOR_POINT, LEFT_BOTTOMRIGHT_ANCHOR_POINT, RED, 5);
                break;
            case CENTER:
                Imgproc.rectangle(input, CENTER_TOPLEFT_ANCHOR_POINT, CENTER_BOTTOMRIGHT_ANCHOR_POINT, GREEN, 5);
                break;
            case RIGHT:
                Imgproc.rectangle(input, RIGHT_TOPLEFT_ANCHOR_POINT, RIGHT_BOTTOMRIGHT_ANCHOR_POINT, BLUE, 5);
                break;
        }

        // convert input to int
        input.convertTo(input, CvType.CV_8U);

        // release the mats
        leftRegionCb.release();
        centerRegionCb.release();
        rightRegionCb.release();
        edges.release();
        hierarchy.release();

        return targetChannel;
    }

    public double[] getAnalysis() {
        return new double[]{leftAvg, centerAvg, rightAvg};
    }

}
