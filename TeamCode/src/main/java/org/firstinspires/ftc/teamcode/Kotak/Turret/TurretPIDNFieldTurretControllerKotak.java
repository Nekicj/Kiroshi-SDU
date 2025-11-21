package org.firstinspires.ftc.teamcode.Kotak.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.Utils.asmPIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Disabled
@TeleOp(name = "Turret Auto aim FIXED", group = "Competition")
public class TurretPIDNFieldTurretControllerKotak extends LinearOpMode {

    private Follower follower;
    private asmGamepadEx driver1;

    private AnalogInput encoder;
    private CRServo turretServo;
    private asmPIDController turretPID;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 5;

    public static double MIN_TURRET_ANGLE = -180;
    public static double MAX_TURRET_ANGLE = 180;
    public static double MIN_CONTINUOUS = -765;
    public static double MAX_CONTINUOUS = 1027;
    public static double ZERO_CONTINUOUS = 123;

    public static double TARGET_X = 10;
    public static double TARGET_Y = 10;

    private double lastRawAngle = 0;
    private int fullRotations = 0;
    private double continuousAngle = 0;

    private double lastContinuousAngle = 0;
    private boolean firstUpdate = true;
    private static final double MAX_REASONABLE_DELTA = 180;

    private boolean autoAimEnabled = true;
    private boolean manualOverride = false;
    private boolean toTarget = false;

    public static double targetTurretAngle = 0;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(0, 0, 0));

        initializeTurret();

        driver1 = new asmGamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            driver1.update();

            driveBase();

            updateContinuousAngle();

            if (autoAimEnabled && !manualOverride) {
                updateAutoAim();
            } else {
                updateManualTurretControl();
            }

            updateModeSwitching();

            showTelemetry();
        }
    }

    private void initializeTurret() {
        turretPID = new asmPIDController(kP, kI, kD);
        turretPID.setTolerance(tolerance);

        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        turretServo = hardwareMap.get(CRServo.class, "turret_r");

        lastRawAngle = getRawAngle();
        continuousAngle = lastRawAngle;
        lastContinuousAngle = continuousAngle;
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

    private void updateAutoAim() {
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();

        double fieldAngleToTarget = calculateFieldAngleToTarget(robotX, robotY, robotHeading);

        double turretTargetAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, fieldAngleToTarget));

        double targetContinuous = turretToContinuousAngle(turretTargetAngle);

        double currentContinuous = getContinuousAngle();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetContinuous);

        double servoPower = turretPID.calculate(currentContinuous);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));

        turretServo.setPower(-servoPower);
    }

    private void updateManualTurretControl() {
        if (!toTarget) {
            if (gamepad1.y) {
                turretServo.setPower(1);
            } else if (gamepad1.b) {
                turretServo.setPower(-1);
            } else {
                turretServo.setPower(0);
            }
        } else {
            double targetContinuous = turretToContinuousAngle(targetTurretAngle);
            double currentContinuous = getContinuousAngle();

            turretPID.setCoefficients(kP, kI, kD);
            turretPID.setTarget(targetContinuous);

            double servoPower = turretPID.calculate(currentContinuous);
            servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
            turretServo.setPower(-servoPower);
        }
    }

    private void updateModeSwitching() {
        if (gamepad1.x) {
            autoAimEnabled = !autoAimEnabled;
            sleep(200);
        }

        if (gamepad1.right_bumper) {
            toTarget = !toTarget;
            if (toTarget) {
                turretPID.reset();
            }
            sleep(200);
        }

        if (gamepad1.left_bumper) {
            manualOverride = true;
        } else {
            manualOverride = false;
        }

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

        if (toTarget && !autoAimEnabled) {
            if (gamepad1.y) {
                targetTurretAngle += 1;
                sleep(100);
            } else if (gamepad1.b) {
                targetTurretAngle -= 1;
                sleep(100);
            }
            targetTurretAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, targetTurretAngle));
        }


        if (gamepad1.back) {
            resetRotationTracking();
            telemetry.addLine("Rotation tracking RESET");
            telemetry.update();
            sleep(500);
        }
    }

    private double calculateFieldAngleToTarget(double robotX, double robotY, double robotHeading) {
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        double absoluteAngleToTarget = Math.atan2(dy, dx);

        double relativeAngle = absoluteAngleToTarget - robotHeading;

        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

        double angleDegrees = Math.toDegrees(relativeAngle);

        return angleDegrees;
    }

    private double continuousToTurretAngle(double continuous) {
        continuous = Math.max(MIN_CONTINUOUS, Math.min(MAX_CONTINUOUS, continuous));

        double totalRange = MAX_CONTINUOUS - MIN_CONTINUOUS;
        double positionFromMin = continuous - MIN_CONTINUOUS;
        double percentage = positionFromMin / totalRange;

        double turretRange = MAX_TURRET_ANGLE - MIN_TURRET_ANGLE;
        return MIN_TURRET_ANGLE + (percentage * turretRange);
    }

    private double turretToContinuousAngle(double turretAngle) {
        turretAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, turretAngle));

        double turretRange = MAX_TURRET_ANGLE - MIN_TURRET_ANGLE;
        double angleFromMin = turretAngle - MIN_TURRET_ANGLE;
        double percentage = angleFromMin / turretRange;

        double continuousRange = MAX_CONTINUOUS - MIN_CONTINUOUS;
        return MIN_CONTINUOUS + (percentage * continuousRange);
    }

    private double getRawAngle() {
        return encoder.getVoltage() * 360.0 / encoder.getMaxVoltage();
    }

    private void updateContinuousAngle() {
        double currentRawAngle = getRawAngle();

        if (firstUpdate) {
            lastRawAngle = currentRawAngle;
            continuousAngle = currentRawAngle;
            lastContinuousAngle = continuousAngle;
            firstUpdate = false;
            return;
        }

        double delta = currentRawAngle - lastRawAngle;


        double deltaFromLastContinuous = Math.abs(continuousAngle - lastContinuousAngle);
        if (deltaFromLastContinuous > MAX_REASONABLE_DELTA) {
//            resetRotationTracking();
            return;
        }

        if (delta > 180) {
            fullRotations--;
        } else if (delta < -180) {
            fullRotations++;
        }

        lastRawAngle = currentRawAngle;
        continuousAngle = currentRawAngle + (fullRotations * 360.0);
        lastContinuousAngle = continuousAngle;
    }


    private void resetRotationTracking() {
        double currentRawAngle = getRawAngle();
        lastRawAngle = currentRawAngle;
        continuousAngle = currentRawAngle;
        lastContinuousAngle = continuousAngle;
        fullRotations = 0;
        firstUpdate = true;
    }

    private double getContinuousAngle() {
        return continuousAngle;
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;
    }

    private void showTelemetry() {
        Pose robotPose = follower.getPose();
        double currentContinuous = getContinuousAngle();
        double currentTurretAngle = continuousToTurretAngle(currentContinuous);
        double fieldAngle = calculateFieldAngleToTarget(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        double limitedFieldAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, fieldAngle));

        telemetry.addLine("=== TURRET AUTO AIM FIXED ===");
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", robotPose.getX(), robotPose.getY());
        telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Target Position", "X: %.1f, Y: %.1f", TARGET_X, TARGET_Y);
        telemetry.addData("Field Angle to Target", "%.1f°", fieldAngle);
        telemetry.addData("Limited Field Angle", "%.1f°", limitedFieldAngle);
        telemetry.addData("Turret Current Angle", "%.1f°", currentTurretAngle);
        telemetry.addData("Continuous Angle", "%.1f°", currentContinuous);
        telemetry.addData("Raw Angle", "%.1f°", getRawAngle());
        telemetry.addData("Full Rotations", fullRotations);

        telemetry.addLine("");
        telemetry.addLine("=== SYSTEM STATUS ===");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Manual Override", manualOverride ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Turret Control", toTarget ? "ANGLE TARGET" : "DIRECT MANUAL");

        if (Math.abs(currentContinuous - MIN_CONTINUOUS) < 50 ||
                Math.abs(currentContinuous - MAX_CONTINUOUS) < 50) {
            telemetry.addLine("*** WARNING: Near physical limit! ***");
        }

        double actualDelta = Math.abs(continuousAngle - lastContinuousAngle);
        telemetry.addData("Actual Delta/Cycle", "%.1f°", actualDelta);
        telemetry.addData("Max Allowed Delta", "%.1f°", MAX_REASONABLE_DELTA);

        if (actualDelta > MAX_REASONABLE_DELTA) {
            telemetry.addLine("*** DELTA LIMIT EXCEEDED ***");
        }

        telemetry.update();
    }
}