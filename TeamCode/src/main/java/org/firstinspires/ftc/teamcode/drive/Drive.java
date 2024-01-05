package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.utils.ActionHandler;
import org.firstinspires.ftc.teamcode.drive.utils.TelemetryUtils;
import org.firstinspires.ftc.teamcode.drive.utils.Vision;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;


/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class Drive {

    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define hardware objects.

    public MecanumDrive drive = null;
    private Vision vision = null;


    // Define Drive constants.
    private final Pose2d INITIAL_POSE = new Pose2d(0, 0, 0);
    private ActionHandler handler;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Drive(LinearOpMode opMode) {
        myOpMode = opMode;
        TelemetryUtils.telemetry = opMode.telemetry;
        this.init();
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        drive = new MecanumDrive(myOpMode.hardwareMap);

        MecanumDrive.PARAMS.kS = DriveConstants.kStatic;
        MecanumDrive.PARAMS.kA = DriveConstants.kA;
        MecanumDrive.PARAMS.kV = DriveConstants.kV;
        MecanumDrive.PARAMS.inPerTick = 1 / DriveConstants.TICKS_PER_INCH;
        MecanumDrive.PARAMS.usbFacingDirection = DriveConstants.USB_FACING_DIR;
        MecanumDrive.PARAMS.logoFacingDirection = DriveConstants.LOGO_FACING_DIR;

        handler = new ActionHandler(myOpMode);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void setMotorPowers(double leftBack, double leftFront, double rightBack, double rightFront) {
        this.drive.leftBack.setPower(leftBack);
        this.drive.leftFront.setPower(leftFront);
        this.drive.rightBack.setPower(rightBack);
        this.drive.rightFront.setPower(rightFront);
    }

    public void updateDrivePoseEstimate() {
        this.drive.updatePoseEstimate();
    }

    public void driveRobot(double drive, double strafe, double turn, double speed) {
        PoseVelocity2d pw = new PoseVelocity2d(
                new Vector2d(
                        -drive * speed,
                        strafe * speed
                ), turn * speed
        );

        this.drive.setDrivePowers(pw);
    }

    public void fieldCentricDriveRobot(double gamepadLeftStickY, double gamepadLeftStickX, double gamepadRightStickX, double speed) {
        updateDrivePoseEstimate();

        Vector2d angleVector = this.drive.pose.heading.vec();
        double angle = -Math.atan2(angleVector.y, angleVector.x);

        double rotatedX = gamepadLeftStickX * Math.cos(angle) - gamepadLeftStickY * Math.sin(angle);
        double rotatedY = gamepadLeftStickX * Math.sin(angle) + gamepadLeftStickY * Math.cos(angle);

        driveRobot(rotatedY, rotatedX, gamepadRightStickX, speed);
    }

    public void setCurrentHeadingAsForward() {
        this.drive.pose = new Pose2d(this.drive.pose.position, Rotation2d.exp(0));
    }

    public PoseVelocity2d alignWithAprilTag(int aprilTagID) {
        Optional<AprilTagDetection> desiredTag = vision.AprilTagDetector.getAprilTagById(aprilTagID);
        if (!desiredTag.isPresent()) {
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }
        AprilTagDetection tag = desiredTag.get();
        double SPEED_GAIN = 0.02;
        double STRAFE_GAIN = 0.01;
        double TURN_GAIN = 0.04;

        double MAX_AUTO_SPEED = 0.5;
        double MAX_AUTO_TURN = 0.3;
        double MAX_AUTO_STRAFE = 0.5;

        double DESIRED_DISTANCE_FROM_APRILTAG = 12.0;
        double rangeError = (tag.ftcPose.range - DESIRED_DISTANCE_FROM_APRILTAG);
        double headingError = tag.ftcPose.bearing;
        double yawError = tag.ftcPose.yaw;

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        return new PoseVelocity2d(new Vector2d(strafe, drive), turn);
    }

    public void setMode(DcMotor.RunMode runMode) {
        this.drive.leftFront.setMode(runMode);
        this.drive.leftBack.setMode(runMode);
        this.drive.rightFront.setMode(runMode);
        this.drive.rightBack.setMode(runMode);
    }

    public Pose2d getINITIAL_POSE() {
        return INITIAL_POSE;
    }

    public void followAsync(Action action) {
        handler.followAsync(action);
    }
    public boolean isBusy() {
        return handler.isBusy();
    }
}