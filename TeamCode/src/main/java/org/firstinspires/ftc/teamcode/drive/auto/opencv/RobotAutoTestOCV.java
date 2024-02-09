/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.auto.opencv;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.auto.CapstoneDetectionCamera;
import org.firstinspires.ftc.teamcode.drive.auto.CapstonePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 * <p>
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 * <p>
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 * <p>
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "!Robot: Auto Test Capstone", group = "Robot")
public class RobotAutoTestOCV extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", 0);
        telemetry.update();

        /* Declare OpMode members. */
        MecanumDrive mecanumDriver = new MecanumDrive(hardwareMap);
        CapstoneDetectionCamera camera = new CapstoneDetectionCamera(hardwareMap, -1);
        CapstonePipeline.CapstonePosition capstonePosition;

        while(opModeInInit()){
            capstonePosition = camera.getPosition();
            telemetry.addData("Current Position",capstonePosition);
            telemetry.addData("Left Analysis",(int) camera.getAnalysis()[0]);
            telemetry.addData("Center Analysis",(int) camera.getAnalysis()[1]);
            telemetry.addData("Right Analysis",(int) camera.getAnalysis()[2]);
            telemetry.addData("Target channel", camera.pipeline.bestChannel);
            telemetry.update();
        }
        // Wait for the game to start (driver presses PLAY)
        TrajectorySequence init = mecanumDriver.trajectorySequenceBuilder(new Pose2d(63.00, -35.00, Math.toRadians(180.00)))
                .splineTo(new Vector2d(54.00, -35.00), Math.toRadians(180.00))
                .build();
        waitForStart();

        try {
            // move an initial distance to the capstone
            mecanumDriver.setPoseEstimate(init.start());
            mecanumDriver.followTrajectorySequence(init);

            // wait for the robot to fully stop
            sleep(1000);
            // get a position
            capstonePosition = camera.getPosition();
            CapstonePipeline.ColorMode channel = camera.pipeline.bestChannel;

            // send telemetry on the position
            telemetry.addData("Detected Position:", capstonePosition);
            telemetry.addData("Using Channel:", channel);
            telemetry.update();

            // move based on the position
            switch (capstonePosition) {
                case LEFT:
                    mecanumDriver.followTrajectorySequence(
                            mecanumDriver.trajectorySequenceBuilder(new Pose2d(-35.00, -63.00, Math.toRadians(90.00)))
                                    .splineTo(new Vector2d(-48.00, -30.00), Math.toRadians(110.00))
                                    .build()
                    );
                case RIGHT:
                    mecanumDriver.followTrajectorySequence(
                            mecanumDriver.trajectorySequenceBuilder(new Pose2d(-35.00, -63.00, Math.toRadians(90.00)))
                                    .splineTo(new Vector2d(-30.00, -36.00), Math.toRadians(0.00))
                                    .build()
                    );
                default:
                    mecanumDriver.followTrajectorySequence(
                            mecanumDriver.trajectorySequenceBuilder(new Pose2d(-35.00, -63.00, Math.toRadians(90.00)))
                                    .splineTo(new Vector2d(-35.00, -24.00), Math.toRadians(90.00))
                                    .build()
                    );
            }

            mecanumDriver.swp0.setPower(0.5);
            mecanumDriver.swp1.setPower(0.5);
        } catch (Exception e) {
            String err = e.toString();

            telemetry.addData("Path failed with exception:", err);
            telemetry.update();

            sleep(5000);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }
}
