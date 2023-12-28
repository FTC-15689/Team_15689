package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = MecanumDrive.pose;

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = drive.rotate(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -poseEstimate.heading.toDouble()
            );

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            input.x,
                            input.y
                    ),
                    -gamepad1.right_stick_x - gamepad1.right_stick_y
            ));
            MecanumDrive.HEADING_PID.p += gamepad1.dpad_up ? 0.5 : gamepad1.dpad_down ? -0.5 : 0;

            // Update everything. Odometry. Etc.
            drive.updatePoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading", poseEstimate.heading.toDouble());
            telemetry.update();
        }
    }
}