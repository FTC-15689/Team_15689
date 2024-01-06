package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 * <p>
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at <a href="https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html">...</a>
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * <p>
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "!Basic: Mecanum Linear OpMode", group = "Linear Opmode")
public class BasicMecanumOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDriver;

    public void robotCenter() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = -gamepad1.right_stick_y + gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        mecanumDriver.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }

    public void fieldCenter() {
        // Read pose
        Pose2d poseEstimate = mecanumDriver.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        mecanumDriver.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
        MecanumDrive.HEADING_PID.kP += gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0;

        // Update everything. Odometry. Etc.
        mecanumDriver.update();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
    }

    public void actions() {
        // conveyor tilt gear ratio: 600:1
        double sweep_speed = gamepad2.right_trigger - gamepad2.left_trigger;
        double targetConveyorAngle = 0;

        if (gamepad2.dpad_right) {
            targetConveyorAngle += 15.0;
        } else if (gamepad2.dpad_left) {
            targetConveyorAngle -= 15.0;
        }

        if (gamepad2.dpad_up) {
            targetConveyorAngle += 5.0;
        } else if (gamepad2.dpad_down) {
            targetConveyorAngle -= 5.0;
        }

        double convRevs = (targetConveyorAngle / 360 * 100);

        if (convRevs != 0.0) {
            mecanumDriver.convAng.setPower(convRevs > 0.0 ? 1.0 : -1.0);
            mecanumDriver.convAng.setTargetPosition((int) (mecanumDriver.convAng.getCurrentPosition() + convRevs));
        } else if (!mecanumDriver.convAng.isBusy()) {
            mecanumDriver.convAng.setPower(0.0);
            mecanumDriver.convAng.setTargetPosition(mecanumDriver.convAng.getCurrentPosition());
        }
        telemetry.addData("Target delta/actual", String.format("%s, %s", convRevs, mecanumDriver.convAng.getTargetPosition()));
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        mecanumDriver = new MecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        mecanumDriver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDriver.convAng.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        mecanumDriver.setPoseEstimate(MecanumDrive.currentPos);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean roboCenter = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                roboCenter = !roboCenter;
            }

            if (roboCenter) {
                robotCenter();
            } else {
                fieldCenter();
            }

            actions();

            telemetry.addLine();
            telemetry.addData("Center:", roboCenter ? "Robot" : "Field");
            telemetry.update();
        }

        // immediately stop all motors
        mecanumDriver.brakeALL();
    }
}
