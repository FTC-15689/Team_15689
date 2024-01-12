package org.firstinspires.ftc.teamcode.drive.auto.simple;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "!PARk: F BLUE", group = "Auto_FrontStage")
public class Park_BLUEF extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", 0);
        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence Park_Blue = drive.trajectorySequenceBuilder(new Pose2d(-39.00, 63.00, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(-39.00, 40.00), Math.toRadians(270.00))
                .splineTo(new Vector2d(-31.00, 36.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.00, 36.00), Math.toRadians(0.00))
                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {
            return;
        }

        drive.followTrajectorySequence(Park_Blue);
        drive.getPoseEst();
    }
}
