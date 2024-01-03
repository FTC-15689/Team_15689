//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;
//
//import android.os.Build;
//
//import androidx.annotation.RequiresApi;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.MovingStatistics;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.internal.system.Misc;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.Drive;
//
//import java.time.Instant;
//import java.util.ArrayList;
//import java.util.List;
//import java.util.Objects;
//
///*
// * Tuning Steps:
// * 1. Feedforward tuner
// * 2. Track-width tuning
// * 3. Turn test
// * 4.
// */
//@Config
//@Autonomous(group = "drive", name = "Mecanum Auto Tuner")
//public class Tuner extends LinearOpMode {
//    private static final int NUM_TRIALS = 5;
//    private static final int ANGLE = 90;
//    private static final long DELAY = (long) 2.0;
//    public static final double MAX_POWER = 0.7;
//    public static final double DISTANCE = 100; // in
//    private double maxAngVelocity = 0.0;
//    private final Pose2d empty_pose = new Pose2d(0, 0, 0);
//
//    @RequiresApi(api = Build.VERSION_CODES.O)
//    @Override
//    public void runOpMode() throws InterruptedException {
//        if (RUN_USING_ENCODER) {
//            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " + "when using the built-in drive motor velocity PID.");
//        }
//
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        Drive drive = new Drive(this);
//
//        telemetry.addLine("Press play to begin the feedforward tuning routine");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        telemetry.clearAll();
//        telemetry.addLine("Would you like to fit kStatic?");
//        telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");
//        telemetry.update();
//
//        boolean fitIntercept = false;
//        while (!isStopRequested() && !gamepad1.b) {
//            if (gamepad1.y) {
//                fitIntercept = true;
//                while (!isStopRequested() && gamepad1.y) {
//                    idle();
//                }
//                break;
//            } else if (gamepad1.b) {
//                while (!isStopRequested() && gamepad1.b) {
//                    idle();
//                }
//                break;
//            }
//            idle();
//        }
//
//        telemetry.clearAll();
//        telemetry.addLine(Misc.formatInvariant("Place your robot on the field with at least %.2f in of room in front", DISTANCE));
//        telemetry.addLine("Press (Y/Δ) to begin");
//        telemetry.update();
//
//        while (!isStopRequested() && !gamepad1.y) {
//            idle();
//        }
//        while (!isStopRequested() && gamepad1.y) {
//            idle();
//        }
//
//        telemetry.clearAll();
//        telemetry.addLine("Running...");
//        telemetry.update();
//
//        double maxVel = rpmToVelocity(MAX_RPM);
//        double finalVel = MAX_POWER * maxVel;
//        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
//        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);
//
//        List<Double> timeSamples = new ArrayList<>();
//        List<Double> positionSamples = new ArrayList<>();
//        List<Double> powerSamples = new ArrayList<>();
//
//        drive.setPoseEstimate(empty_pose);
//
//        double startTime = Instant.now().getEpochSecond();
//        while (!isStopRequested() && !gamepad1.b) {
//            double elapsedTime = Instant.now().getEpochSecond() - startTime;
//            if (elapsedTime > rampTime) {
//                break;
//            }
//            double vel = accel * elapsedTime;
//            double power = vel / maxVel;
//
//            timeSamples.add(elapsedTime);
//            positionSamples.add(drive.getPoseEstimate().position.x);
//            powerSamples.add(power);
//
//            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, power), 0.0));
//            drive.updatePoseEstimate();
//        }
//        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));
//
//        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(timeSamples, positionSamples, powerSamples, fitIntercept, LoggingUtil.getLogFile(Misc.formatInvariant("DriveRampRegression-%d.csv", System.currentTimeMillis())));
//
//        telemetry.clearAll();
//        telemetry.addLine("Quasi-static ramp up test complete");
//        if (fitIntercept) {
//            telemetry.addLine(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)", rampResult.kV, rampResult.kStatic, rampResult.rSquare));
//            kV = rampResult.kV;
//        } else {
//            telemetry.addLine(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)", rampResult.kStatic, rampResult.rSquare));
//        }
//        kStatic = rampResult.kStatic;
//        telemetry.addLine("Would you like to fit kA?");
//        telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");
//        telemetry.update();
//
//        boolean fitAccelFF = false;
//        while (!isStopRequested() && !gamepad1.b) {
//            if (gamepad1.y) {
//                fitAccelFF = true;
//                while (!isStopRequested() && gamepad1.y) {
//                    idle();
//                }
//                break;
//            } else if (gamepad1.b) {
//                while (!isStopRequested() && gamepad1.b) {
//                    idle();
//                }
//                break;
//            }
//            idle();
//        }
//
//        if (fitAccelFF) {
//            telemetry.clearAll();
//            telemetry.addLine("Place the robot back in its starting position");
//            telemetry.addLine("Press (Y/Δ) to continue");
//            telemetry.update();
//
//            while (!isStopRequested() && !gamepad1.y) {
//                idle();
//            }
//            while (!isStopRequested() && gamepad1.y) {
//                idle();
//            }
//
//            telemetry.clearAll();
//            telemetry.addLine("Running...");
//            telemetry.update();
//
//            double maxPowerTime = DISTANCE / maxVel;
//
//            timeSamples.clear();
//            positionSamples.clear();
//            powerSamples.clear();
//
//            drive.setPoseEstimate();
//            drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));
//
//            startTime = Instant.now().getEpochSecond();
//            while (!isStopRequested() && !gamepad1.b) {
//                double elapsedTime = Instant.now().getEpochSecond() - startTime;
//                if (elapsedTime > maxPowerTime) {
//                    break;
//                }
//
//                timeSamples.add(elapsedTime);
//                positionSamples.add(drive.getPoseEstimate().position.x);
//                powerSamples.add(MAX_POWER);
//
//                drive.updatePoseEstimate();
//            }
//            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));
//
//            RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(timeSamples, positionSamples, powerSamples, rampResult, LoggingUtil.getLogFile(Misc.formatInvariant("DriveAccelRegression-%d.csv", System.currentTimeMillis())));
//
//            telemetry.clearAll();
//            telemetry.addLine("Constant power test complete");
//            telemetry.addLine(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)", accelResult.kA / 10, accelResult.rSquare));
//            telemetry.update();
//            kA = accelResult.kA / 10;
//        }
//
//        // Next: Track width tuning
//
//        telemetry.addLine("Press Y to begin the track width tuner routine or B to skip");
//        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
//        telemetry.update();
//
//        while (!gamepad1.y || !gamepad1.b) {
//            idle();
//        }
//
//        if (isStopRequested()) return;
//
//        if (gamepad1.y) {
//            telemetry.clearAll();
//            telemetry.addLine("Running...");
//            telemetry.update();
//
//            MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
//            for (int i = 0; i < NUM_TRIALS; i++) {
//                drive.setPoseEstimate();
//
//                // it is important to handle heading wraparounds
//                double headingAccumulator = 0;
//                double lastHeading = 0;
//
//                drive.turnAsync(Math.toRadians(ANGLE));
//
//                while (!isStopRequested() && drive.isBusy()) {
//                    double heading = drive.getPoseEstimate().heading.toDouble();
//                    double deltaHeading = heading - lastHeading;
//                    headingAccumulator += (deltaHeading % (Math.PI * 2)) % (Math.PI * 2);
//                    lastHeading = heading;
//
//                    drive.updatePoseEstimate();
//                }
//
//                double trackWidth = DriveConstants.TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator;
//                trackWidthStats.add(trackWidth);
//
//                sleep(DELAY);
//            }
//
//            telemetry.clearAll();
//            telemetry.addLine("Tuning complete");
//            telemetry.addLine(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)", trackWidthStats.getMean(), trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
//            telemetry.update();
//
//            TRACK_WIDTH = trackWidthStats.getMean();
//        }
//
//        // Next: Max ANG Velocity
//        telemetry.addLine("Press Y to find the max angular velocity or B to skip");
//        telemetry.update();
//
//        while (!gamepad1.y || !gamepad1.b) {
//            idle();
//        }
//
//        if (gamepad1.y) {
//            drive.setDrivePower(new Pose2d(0, 0, 1));
//            ElapsedTime timer = new ElapsedTime();
//
//            double RUNTIME = 10.0;
//            while (!isStopRequested() && timer.seconds() < RUNTIME && !gamepad1.b) {
//                drive.updatePoseEstimate();
//
//                PoseVelocity2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
//
//                maxAngVelocity = Math.max(poseVelo.angVel, maxAngVelocity);
//                MAX_ANG_VEL = maxAngVelocity;
//            }
//
//            drive.setDrivePower(new Pose2d(0,0,0));
//
//            telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity);
//            telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity));
//            telemetry.addData("Max Recommended Angular Velocity (rad)", maxAngVelocity * 0.8);
//            telemetry.addData("Max Recommended Angular Velocity (deg)", Math.toDegrees(maxAngVelocity * 0.8));
//            telemetry.update();
//        }
//
//        // Next: Back and forth
//
//        telemetry.clearAll();
//        telemetry.addLine("Press Y to begin back and forth tuning or B to skip");
//        telemetry.update();
//
//        Action trajectoryForward = drive.actionBuilder(drive.getPoseEstimate())
//                .lineToX(DISTANCE)
//                .build();
//
//        Action trajectoryBackward = drive.actionBuilder(new Pose2d(drive.getPoseEstimate().position.minus(new Vector2d(DISTANCE, 0)), 0))
//                .lineToX(-DISTANCE)
//                .build();
//
//        while (!gamepad1.y || !gamepad1.b) {
//            idle();
//        }
//
//        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
//            drive.followAction(trajectoryForward);
//            drive.followAction(trajectoryBackward);
//        }
//
//        while (!isStopRequested()) {
//            idle();
//        }
//    }
//}