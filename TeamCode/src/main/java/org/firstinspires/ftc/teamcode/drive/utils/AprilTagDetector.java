package org.firstinspires.ftc.teamcode.drive.utils;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class AprilTagDetector {
    // Backdrop April Tag IDs
    // Blue Left: 1, Blue Center: 2, Blue Right: 3
    // Red Left: 4, Red Center 5, Red Right: 6

    public AprilTagProcessor aprilTag;

    public AprilTagDetector() {
        aprilTag = new AprilTagProcessor.Builder().setDrawTagOutline(true).build();
    }

    public Optional<AprilTagDetection> closestAprilTag() {
        // Step through the list of detected tags and look for closest one.
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty())
            return Optional.empty();


        AprilTagDetection closestTag = currentDetections.get(0);

        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null)) {
                if (detection.ftcPose.range < closestTag.ftcPose.range) {
                    closestTag = detection;
                }
            }
        }

        return Optional.of(closestTag);
    }

    public Optional<AprilTagDetection> getAprilTagById(int id) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == id) {
                    return Optional.of(detection);
                }
            }
        }
        return Optional.empty();
    }

    public List<AprilTagDetection> getWallAprilTags() {
        return aprilTag.getDetections().stream().filter(detection -> detection.metadata.name.contains("Wall")).collect(Collectors.toList());
    }

    public List<AprilTagDetection> getBackdropAprilTags() {
        return aprilTag.getDetections().stream().filter(detection -> detection.metadata.name.contains("Alliance")).collect(Collectors.toList());
    }
}
