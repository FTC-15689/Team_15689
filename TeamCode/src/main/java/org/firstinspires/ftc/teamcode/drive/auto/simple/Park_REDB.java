package org.firstinspires.ftc.teamcode.drive.auto.simple;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "!PARk: B RED", group = "Auto_BackStage")
public class Park_REDB extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Send a telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", 0);
        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence Park_Red = drive.trajectorySequenceBuilder(new Pose2d(15.00, -63.00, Math.toRadians(450.00)))
                .splineTo(new Vector2d(45.00, -36.00), Math.toRadians(360.00))
                .build();

        TrajectorySequence Park_Red2 = drive.trajectorySequenceBuilder(new Pose2d(45.00, -36.00, Math.toRadians(360.00)))
                .splineToConstantHeading(new Vector2d(20.00, -36.00), Math.toRadians(360.00))
                .splineToConstantHeading(new Vector2d(20.00, -12.00), Math.toRadians(360.00))
                .splineToConstantHeading(new Vector2d(60.00, -12.00), Math.toRadians(360.00))
                .build();

        drive.setPoseEstimate(Park_Red.start());

        // Wait for the game to start (a driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {return;}

        drive.followTrajectorySequence(Park_Red);
        drive.getPoseEst();

        drive.setPoseEstimate(Park_Red.end());

        // spit out the two pixels
        drive.swp0.setPower(-1);
        drive.swp1.setPower(1);
        drive.swp2.setPower(-0.6);

        sleep(2000);

        drive.swp0.setPower(0);
        drive.swp1.setPower(0);
        drive.swp2.setPower(0);

        drive.setPoseEstimate(Park_Red2.start());
        drive.followTrajectorySequence(Park_Red2);
    }
}
