package org.firstinspires.ftc.teamcode.drive.auto.simple;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Drive;

@Autonomous(name = "!PARk: RED", group = "Robot")
public class Park_RED extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", 0);
        telemetry.update();

        Drive mecanumDriver = new Drive(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {return;}

        Actions.runBlocking(
                mecanumDriver.drive.actionBuilder(
                                new Pose2d(63.00, -35.00, Math.toRadians(180.00))
                        )
                        .splineTo(new Vector2d(12.00, -35.00), Math.toRadians(180.00))
                        .splineTo(new Vector2d(12.00, 63.0), Math.toRadians(180.00))
                        .build()
        );

//        while (mecanumDriver.isBusy()) {
//            idle();
//        }
//
//        telemetry.addLine("Finished parking.");
//        telemetry.update();
    }
}
