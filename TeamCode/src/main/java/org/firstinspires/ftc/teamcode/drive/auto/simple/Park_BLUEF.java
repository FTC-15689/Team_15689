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
                .splineTo(new Vector2d(-50.00, 36.00), Math.toRadians(-90.00))
                .lineToLinearHeading(new Pose2d(-50.00, 10.00, Math.toRadians(180.00)))
                .build();

        TrajectorySequence Park_Blue1 = drive.trajectorySequenceBuilder(new Pose2d(-50.00, 10.00, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(15.00, 10.00))
                .lineToSplineHeading(new Pose2d(15.00, 18.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(45.00, 36.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence Park_Blue2 = drive.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0.00)))
                .splineToConstantHeading(new Vector2d(20.00, 36.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(20.00, 12.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(60.00, 12.00), Math.toRadians(0.00))
                .build();

        drive.setPoseEstimate(Park_Blue.start());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {
            return;
        }

        drive.followTrajectorySequence(Park_Blue);
        drive.getPoseEst();

        // set the conveyor down
        int target_ticks = (int) (3500.0 * 90.0 / 360.0);
        drive.convAng.setTargetPosition(drive.convAng.getCurrentPosition() - target_ticks);
        while (drive.convAng.isBusy()) {
            drive.convAng.setPower(-0.5);
        }
        drive.convAng.setPower(0);

        drive.followTrajectorySequence(Park_Blue1);
        drive.getPoseEst();

        // spit out the two pixels
        drive.swp0.setPower(-1);
        drive.swp1.setPower(1);
        drive.swp2.setPower(-0.6);

        sleep(2000);

        drive.swp0.setPower(0);
        drive.swp1.setPower(0);
        drive.swp2.setPower(0);

        drive.setPoseEstimate(Park_Blue2.start());
        drive.followTrajectorySequence(Park_Blue2);
    }
}
