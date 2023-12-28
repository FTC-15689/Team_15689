package org.firstinspires.ftc.teamcode.drive.auto.simple;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "!PARk: BLUE", group = "Robot")
public class Park_BLUE extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", 0);
        telemetry.update();

        Pose2d empty_pose = new Pose2d(0, 0, 0);
        MecanumDrive mecanumDriver = new MecanumDrive(hardwareMap, empty_pose);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        mecanumDriver.setPoseEstimate(startPose);

        Action Park_Blue = mecanumDriver.genPath(1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {
            return;
        }

        mecanumDriver.followAction(Park_Blue);
        mecanumDriver.updatePoseEstimate();
    }
}
