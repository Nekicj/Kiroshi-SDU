package org.firstinspires.ftc.teamcode.Kotak.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.ShooterControllerPIDVSA;
import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Auto Shooter Velocity with PIDVSA", group = "Tuning")
public class AutoShooterVelocityControl extends LinearOpMode {

    public static double TARGET_X = 0;
    public static double TARGET_Y = 0;
    public static double TARGET_HEIGHT = 50;
    public static double LAUNCH_HEIGHT = 10;
    public static double LAUNCH_ANGLE = 45;

    public static double GRAVITY = 980;

    public static double WHEEL_DIAMETER = 10.0;
    public static double MAX_RPM = 2100;
    public static double MIN_RPM = 500;

    public static double SHOOT_READY_TOLERANCE = 50;

    private Follower follower;
    private asmGamepadEx driver1;
    private ShooterControllerPIDVSA shooterController;

    private boolean autoShooterEnabled = true;
    private boolean shooterActive = false;
    private double currentTargetRPM = 0;
    private double calculatedRPM = 0;

    @Override
    public void runOpMode() {
        initializeSystems();

        waitForStart();

        while (opModeIsActive()) {
            driver1.update();

            driveBase();


            shooterController.update();

            if (autoShooterEnabled) {
                updateAutoShooter();
            }

            updateManualShooterControl();

            updateModeSwitching();

            showTelemetry();
        }
    }

    private void initializeSystems() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(0, 0, 0));

        shooterController = new ShooterControllerPIDVSA();
        shooterController.initialize(
                hardwareMap,
                "shooter_left",
                "shooter_right",
                "servo_angle_left",
                "servo_angle_right",
                0.5
        );

        driver1 = new asmGamepadEx(gamepad1);
    }

    private void driveBase() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        forward = applyDeadzone(forward, 0.05);
        strafe = applyDeadzone(strafe, 0.05);
        rotate = applyDeadzone(rotate, 0.05);

        follower.update();
        follower.setTeleOpDrive(forward, strafe, rotate * 0.7, false);
    }

    private void updateAutoShooter() {
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double distance = calculateDistanceToTarget(robotX, robotY);

        double requiredVelocity = calculateRequiredVelocity(distance);

        calculatedRPM = convertVelocityToRPM(requiredVelocity);

        calculatedRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, calculatedRPM));
        currentTargetRPM = calculatedRPM;

        if (shooterActive) {
            shooterController.setShooterVelocity(currentTargetRPM);
        } else {
            shooterController.setShooterVelocity(0);
        }
    }

    private double calculateDistanceToTarget(double robotX, double robotY) {
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }


    private double calculateRequiredVelocity(double distance) {
        double angleRad = Math.toRadians(LAUNCH_ANGLE);

        double heightDifference = TARGET_HEIGHT - LAUNCH_HEIGHT;

        double cosAngle = Math.cos(angleRad);
        double sinAngle = Math.sin(angleRad);

        double denominator = 2 * cosAngle * cosAngle * (heightDifference + distance * Math.tan(angleRad));

        if (denominator <= 0) {
            return convertRPMToVelocity(MAX_RPM);
        }

        double velocitySquared = (GRAVITY * distance * distance) / denominator;

        if (velocitySquared <= 0) {
            return convertRPMToVelocity(MIN_RPM);
        }

        return Math.sqrt(velocitySquared);
    }

    private double convertVelocityToRPM(double velocityCmPerSec) {
        double circumference = Math.PI * WHEEL_DIAMETER;

        double rotationsPerSecond = velocityCmPerSec / circumference;

        return rotationsPerSecond * 60.0;
    }

    private double convertRPMToVelocity(double rpm) {
        double circumference = Math.PI * WHEEL_DIAMETER;

        double rotationsPerSecond = rpm / 60.0;

        return rotationsPerSecond * circumference;
    }

    private void updateManualShooterControl() {
        if (!autoShooterEnabled) {
            if (shooterActive) {
                if (gamepad1.dpad_up) {
                    currentTargetRPM += 50;
                    sleep(100);
                } else if (gamepad1.dpad_down) {
                    currentTargetRPM -= 50;
                    sleep(100);
                }

                currentTargetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, currentTargetRPM));


                shooterController.setShooterVelocity(currentTargetRPM);
            } else {
                shooterController.setShooterVelocity(0);
            }
        }
    }

    private void updateModeSwitching() {
        if (gamepad1.a) {
            shooterActive = !shooterActive;
            if (!shooterActive) {
                shooterController.setShooterVelocity(0);
            }
            sleep(200);
        }

        if (gamepad1.x) {
            autoShooterEnabled = !autoShooterEnabled;
            sleep(200);
        }

        if (gamepad1.right_bumper) {
            double currentPos = ShooterControllerPIDVSA.ServosPos.DIRECTION_UP.getPos();
            if (gamepad1.dpad_up) {
                currentPos += 0.01;
                sleep(100);
            } else if (gamepad1.dpad_down) {
                currentPos -= 0.01;
                sleep(100);
            }
            currentPos = Math.max(0, Math.min(1, currentPos));
            shooterController.setDirectionPos(currentPos);
        }

        if (gamepad1.left_bumper) {
            if (gamepad1.dpad_up) {
                TARGET_Y += 10;
                sleep(200);
            } else if (gamepad1.dpad_down) {
                TARGET_Y -= 10;
                sleep(200);
            }
            if (gamepad1.dpad_right) {
                TARGET_X += 10;
                sleep(200);
            } else if (gamepad1.dpad_left) {
                TARGET_X -= 10;
                sleep(200);
            }
        }

        if (gamepad1.y) {
            TARGET_HEIGHT += 5;
            sleep(200);
        } else if (gamepad1.b) {
            TARGET_HEIGHT -= 5;
            sleep(200);
        }
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;
    }

    private void showTelemetry() {
        Pose robotPose = follower.getPose();
        double distance = calculateDistanceToTarget(robotPose.getX(), robotPose.getY());
        double requiredVelocity = calculateRequiredVelocity(distance);
        double calculatedRPMFromVelocity = convertVelocityToRPM(requiredVelocity);

        telemetry.addLine("=== AUTO SHOOTER WITH PIDVSA CONTROLLER ===");
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", robotPose.getX(), robotPose.getY());
        telemetry.addData("Target Position", "X: %.1f, Y: %.1f", TARGET_X, TARGET_Y);
        telemetry.addData("Distance to Target", "%.1f cm", distance);
        telemetry.addData("Target Height", "%.1f cm", TARGET_HEIGHT);
        telemetry.addData("Launch Angle", "%.1f°", LAUNCH_ANGLE);

        telemetry.addLine("");
        telemetry.addLine("=== VELOCITY CALCULATIONS ===");
        telemetry.addData("Required Velocity", "%.1f cm/s", requiredVelocity);
        telemetry.addData("Calculated RPM", "%.1f", calculatedRPMFromVelocity);
        telemetry.addData("Current Target RPM", "%.1f", currentTargetRPM);
        telemetry.addData("Ready to Shoot", shooterController.isReadyToShoot(SHOOT_READY_TOLERANCE) ? "YES" : "NO");

        telemetry.addLine("");
        telemetry.addLine("=== SYSTEM STATUS ===");
        telemetry.addData("Auto Shooter", autoShooterEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Shooter Active", shooterActive ? "ACTIVE" : "INACTIVE");
//        telemetry.addData("Servo Position", "%.3f", shooterController.directionPos);


        shooterController.showTelemetry(telemetry);

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Toggle Shooter", "A button");
        telemetry.addData("Toggle Auto/Manual", "X button");
        telemetry.addData("Adjust Angle (RB + DPad)", "Change shooter angle");
        telemetry.addData("Move Target (LB + DPad)", "Change target position");
        telemetry.addData("Change Height", "Y/B buttons");

        if (!autoShooterEnabled) {
            telemetry.addData("Manual RPM Control", "DPad Up/Down");
        }

        telemetry.update();
    }
}