package org.firstinspires.ftc.teamcode.drive.auto.simple;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "!PARk: F RED", group = "Auto_FrontStage")
public class Park_REDF extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", 0);
        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence Park_Red = drive.trajectorySequenceBuilder(new Pose2d(-39.00, -63.00, Math.toRadians(450.00)))
                .splineTo(new Vector2d(-39.00, -20.00), Math.toRadians(450.00))
                .splineTo(new Vector2d(-20.00, -12.00), Math.toRadians(360.00))
                .lineTo(new Vector2d(14.14, -11.94))
                .lineTo(new Vector2d(49.88, -17.36))
                .build();

        drive.setPoseEstimate(Park_Red.start());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {
            return;
        }

        drive.followTrajectorySequence(Park_Red);
        drive.getPoseEst();
    }
}
