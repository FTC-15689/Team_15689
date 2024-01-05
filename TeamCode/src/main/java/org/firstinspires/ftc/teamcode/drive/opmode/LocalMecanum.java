package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `OldMecanumDrive.md`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced", name = "!Localized Mecanum Drive")
public class LocalMecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize OldMecanumDrive
        Drive drive = new Drive(this);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.fieldCentricDriveRobot(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x + -gamepad1.right_stick_y,
                    DriveConstants.MAX_VEL / 2
            );

            // Print pose to telemetry
            Pose2d poseEstimate = drive.drive.pose;
            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading", poseEstimate.heading.toDouble());
            telemetry.update();
        }
    }
}