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

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderInchesToTicks;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.auto.CapstoneDetectionCamera;
import org.firstinspires.ftc.teamcode.drive.auto.CapstonePipeline;

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

    /* Declare OpMode members. */
    private MecanumDrive mecanumDriver;
    private final ElapsedTime runtime = new ElapsedTime();
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", 0);
        telemetry.update();

        mecanumDriver = new MecanumDrive(hardwareMap);
        CapstoneDetectionCamera camera = new CapstoneDetectionCamera(hardwareMap, -1);
        CapstonePipeline.CapstonePosition capstonePosition;

        while(opModeInInit()){
            capstonePosition = camera.getPosition();
            telemetry.addData("Current Position",capstonePosition);
            telemetry.addData("Left Analysis",camera.getAnalysis()[0]);
            telemetry.addData("Center Analysis",camera.getAnalysis()[1]);
            telemetry.addData("Right Analysis",camera.getAnalysis()[2]);
            telemetry.update();
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        try {
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            encoderDrive(DRIVE_SPEED, 10, 10);  // S1: Forward 47 Inches
            encoderDrive(TURN_SPEED, 12, -12);  // S2: Turn Right 12 Inches
            encoderDrive(DRIVE_SPEED, -10, -10);  // S3: Reverse 24 Inches
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

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches
                             ) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // get how many encoder ticks to move (in/in * ticks)
            int ticksToMoveLeft = encoderInchesToTicks(leftInches);
            int ticksToMoveRight = encoderInchesToTicks(rightInches);

            telemetry.addData("Moving by", "%d %d", ticksToMoveLeft, ticksToMoveRight);
            telemetry.update();

            mecanumDriver.goToEncoderPositionREL(
                    ticksToMoveLeft,
                    ticksToMoveLeft,
                    ticksToMoveRight,
                    ticksToMoveRight,
                    speed
            );

            mecanumDriver.waitForIdle();

            mecanumDriver.setMotorPowers(0, 0, 0, 0);
        }
        mecanumDriver.brakeALL();

        sleep(250);
    }
}
