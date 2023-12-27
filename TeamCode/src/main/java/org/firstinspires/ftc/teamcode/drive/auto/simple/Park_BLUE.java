package org.firstinspires.ftc.teamcode.drive.auto.simple;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OldMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "!PARk: BLUE", group = "Robot")
public class Park_BLUE extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", 0);
        telemetry.update();

        OldMecanumDrive mecanumDriver = new OldMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        mecanumDriver.setPoseEstimate(startPose);

        TrajectorySequence Park_Blue = mecanumDriver.genPath(1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {return;}

        mecanumDriver.followTrajectorySequence(Park_Blue);
        mecanumDriver.getPoseEst();
    }
}
