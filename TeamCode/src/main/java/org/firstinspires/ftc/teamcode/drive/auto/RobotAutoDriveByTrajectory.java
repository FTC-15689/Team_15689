package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous(name="!Robot: Auto Drive By Traject", group="Robot")
public class RobotAutoDriveByTrajectory extends LinearOpMode {
    private MecanumDrive mecanumDriver;
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", 0);
        telemetry.update();

        mecanumDriver = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        mecanumDriver.setPoseEstimate(startPose);

        // paths
//        TrajectorySequence Park_Blue = mecanumDriver.trajectorySequenceBuilder(new Pose2d(61.00, -36.00, Math.toRadians(180.00)))
//                .splineTo(new Vector2d(9.10, -35.31), Math.toRadians(179.24))
//                .lineTo(new Vector2d(10.70, 64.57))
//                .build();
        TrajectorySequence Park_Blue = mecanumDriver.trajectorySequenceBuilder(new Pose2d(-63.00, -34.00, Math.toRadians(0.00)))
                .splineTo(new Vector2d(-13.18, -34.00), Math.toRadians(0.16))
                .lineTo(new Vector2d(-14.20, 69.52))
                .build();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {return;}

        mecanumDriver.followTrajectorySequence(Park_Blue);
    }
}
