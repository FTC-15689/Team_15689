package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "!0: Teleop-Main", group = "Linear Opmode")
public class Main_Teleop extends LinearOpMode {

    public static final double HIGH_RAMP = 0.9;
    public static final double LOW_RAMP = 0.62;
    private static final boolean USE_WEBCAM = true;
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDriver;
    private boolean driveModeBtnDown = false;
    private int paperState = 0;
    private boolean paperBtnDown = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public double bounded(double num, double low, double high) {
        return Math.min(high, Math.max(low, num));
    }

    public void robotCenter() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y - gamepad1.right_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = (gamepad1.right_stick_x) * 0.75;

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

    @SuppressLint("DefaultLocale")
    public void actions() {
        // conveyor tilt gear ratio: 125:1
        // 28 ticks per motor rev
        // 3500 ticks per 360 degrees
        double sweep_speed = gamepad2.right_trigger - gamepad2.left_trigger + gamepad1.right_trigger - gamepad1.left_trigger;

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

        double convRevs = (targetConveyorAngle / 360.0) * 3500.0 + mecanumDriver.convAng.getCurrentPosition();

        if (convRevs != 0.0) {
            mecanumDriver.convAng.setPower(Math.signum(convRevs) * 0.35);
            mecanumDriver.convAng.setTargetPosition((int) (convRevs));
        } else if (!mecanumDriver.convAng.isBusy()) {
            mecanumDriver.convAng.setPower(0.0);
            mecanumDriver.convAng.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mecanumDriver.convAng.setTargetPosition(mecanumDriver.convAng.getCurrentPosition());
        }
        telemetry.addData(
                "Target Degrees, Raw, Remaining\n",
                String.format(
                        "%s, %s, %s",
                        targetConveyorAngle,
                        convRevs,
                        mecanumDriver.convAng.getTargetPosition() - mecanumDriver.convAng.getCurrentPosition()
                )
        );

        // servos
        // ramp thingy
        if (gamepad2.a) { // low
            mecanumDriver.ramp.setPosition(LOW_RAMP);
        } else if (gamepad2.b) { // high
            mecanumDriver.ramp.setPosition(HIGH_RAMP);
        }
        telemetry.addData("Ramp", mecanumDriver.ramp.getPosition() >= LOW_RAMP + (HIGH_RAMP - LOW_RAMP) / 2.0 ? "High" : "Low");
        // sweeper
        if (sweep_speed != 0.0) {
            mecanumDriver.swp2.setPower(bounded(sweep_speed, -0.75, 0.75));
            if (mecanumDriver.ramp.getPosition() <= LOW_RAMP + (HIGH_RAMP - LOW_RAMP) / 2.0) {
                mecanumDriver.swp0.setPower(bounded(sweep_speed, -1.0, 1.0));
                mecanumDriver.swp1.setPower(bounded(-sweep_speed, -1.0, 1.0));
            } else {
                mecanumDriver.swp0.setPower(0.0);
                mecanumDriver.swp1.setPower(0.0);
            }
        } else {
            mecanumDriver.swp0.setPower(0.0);
            mecanumDriver.swp1.setPower(0.0);
            mecanumDriver.swp2.setPower(0.0);
        }
        telemetry.addData("Sweep speed:", (sweep_speed == 0.0) ? "Stopped" : (sweep_speed > 0.0 ? "Sweeping" : "Reversing"));

        // hanging
        double hanger_power = (gamepad1.x ? 1 : 0) - (gamepad1.y ? 1 : 0);
        if (Math.signum(hanger_power) != 0) {
            mecanumDriver.hanger0.setPower(hanger_power);
            mecanumDriver.hanger1.setPower(hanger_power);
        }
        else {
            mecanumDriver.hanger0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mecanumDriver.hanger1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mecanumDriver.hanger0.setPower(0.0);
            mecanumDriver.hanger1.setPower(0.0);
        }
        telemetry.addData("Hanger:", Math.signum(hanger_power) == 0 ? "Hold Pos" : (Math.signum(hanger_power) > 0 ? "Lifting" : "Lowering"));

        // paper plane
        if (!gamepad2.y) {
            paperBtnDown = false;
        } else if (gamepad2.y && !paperBtnDown) {
            paperState += 1;
            paperBtnDown = true;
        }
        if (gamepad2.x) {
            paperState = 0;
        }

        if (paperState > 1) {
            mecanumDriver.paperLcr.setPosition(0.0);
        } else {
            mecanumDriver.paperLcr.setPosition(1.0);
        }
        telemetry.addData("Paper Launcher:", paperState == 0 ? "..." : (paperState == 1 ? "!! ARMED !!" : "Launched"));

        // conveyor movement
        mecanumDriver.conv.setPower(gamepad2.right_stick_x);
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        mecanumDriver = new MecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        mecanumDriver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mecanumDriver.convAng.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        mecanumDriver.setPoseEstimate(MecanumDrive.currentPos);

        if (hardwareMap.tryGet(WebcamName.class, "Webcam 1") != null) {
            initAprilTag();
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        mecanumDriver.convAng.setTargetPosition(0);
        mecanumDriver.convAng.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanumDriver.convAng.setVelocity(1);

        boolean roboCenter = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            if (!gamepad1.dpad_up) {
                driveModeBtnDown = false;
            }
            if (gamepad1.dpad_up && !driveModeBtnDown) {
                roboCenter = !roboCenter;
                driveModeBtnDown = true;
            }
            if (gamepad1.dpad_down) {
                // reset the heading
                Pose2d cur = mecanumDriver.getPoseEstimate();
                mecanumDriver.setPoseEstimate(new Pose2d(cur.getX(), cur.getY(), 0.0));
            }

            if (roboCenter) {
                robotCenter();
            } else {
                fieldCenter();
            }

            actions();
            telemetryAprilTag();

            String cross = "\n \\   /\n  \\ /\n  / \\\n /   \\";
            // \   /
            //  \ /
            //  / \
            // /   \
            String circle = "\n/---\\\n|     |\n\\---/";
            // /---\
            // |   |
            // \---/

            telemetry.addLine();
            telemetry.addData("Center:", roboCenter ? circle : cross);
            telemetry.update();
        }

        // immediately stop all motors
        mecanumDriver.brakeALL();
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}
