package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.asmPIDController;

@Config
public class TurretController {
    private AnalogInput encoder;
    private CRServo turretServo;
    private CRServo turretServoLeft;
    private asmPIDController turretPID;

    private Pose thisRobotPose = new Pose(0,0,0);

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 5;

    public static double MIN_TURRET_ANGLE = -180;
    public static double MAX_TURRET_ANGLE = 180;
    public static double MIN_CONTINUOUS = -517;
    public static double MAX_CONTINUOUS = 717;
    public static double ZERO_CONTINUOUS = 123;

    public static double TARGET_X = -8.87;
    public static double TARGET_Y = 33.91;

    private double lastRawAngle = 0;
    private int fullRotations = 0;
    private double continuousAngle = 0;
    private double lastContinuousAngle = 0;
    private boolean firstUpdate = true;
    private static final double MAX_REASONABLE_DELTA = 180;


    public enum TurretMode {
        FIELD_TARGET,
        FIELD_ANGLE,
        ROBOT_RELATIVE
    }

    private TurretMode currentMode = TurretMode.FIELD_TARGET;
    private boolean autoAimEnabled = true;
    private double targetTurretAngle = 0;
    private double manualPower = 0;

    private double fieldAngleTarget = 0;

    private boolean calibrationMode = false;
    private double calibrationJoystickInput = 0;

    private Gamepad gamepad;
    private boolean lastLeftTriggerState = false;

    public void initialize(HardwareMap hardwareMap, String encoderName, String turretServoName) {
        turretPID = new asmPIDController(kP, kI, kD);
        turretPID.setTolerance(tolerance);

        encoder = hardwareMap.get(AnalogInput.class, encoderName);
        turretServo = hardwareMap.get(CRServo.class, turretServoName);
        turretServoLeft = hardwareMap.get(CRServo.class,"turret_l");

        lastRawAngle = getRawAngle();
        continuousAngle = lastRawAngle;
        lastContinuousAngle = continuousAngle;
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update(Pose robotPose) {
        this.thisRobotPose = robotPose;

        updateContinuousAngle();
        processCalibrationInput();

        if (calibrationMode) {
            turretServo.setPower(calibrationJoystickInput);
            turretServoLeft.setPower(calibrationJoystickInput);
        } else {
            switch (currentMode) {
                case FIELD_TARGET:
                    if (autoAimEnabled) {
                        updateFieldTargetMode(robotPose);
                    } else {
                        updateManualToTarget();
                    }
                    break;
                case FIELD_ANGLE:
                    updateFieldAngleMode();
                    break;
                case ROBOT_RELATIVE:
                    updateRobotRelativeMode();
                    break;
            }
        }
    }

    public void update() {
        updateContinuousAngle();
        processCalibrationInput();

        if (calibrationMode) {
            turretServo.setPower(calibrationJoystickInput);
            turretServoLeft.setPower(calibrationJoystickInput);
        } else {
            switch (currentMode) {
                case FIELD_TARGET:
                    if (autoAimEnabled) {
                        updateFieldTargetMode();
                    } else {
                        updateManualToTarget();
                    }
                    break;
                case FIELD_ANGLE:
                    updateFieldAngleMode();
                    break;
                case ROBOT_RELATIVE:
                    updateRobotRelativeMode();
                    break;
            }
        }
    }

    private void processCalibrationInput() {
        if (gamepad == null) return;

        boolean currentLeftTriggerState = gamepad.left_trigger > 0.1;
        if (currentLeftTriggerState && !lastLeftTriggerState) {
            calibrationMode = true;
            calibrationJoystickInput = -gamepad.right_stick_x;
            resetRotationTracking();
        } else if (!currentLeftTriggerState && lastLeftTriggerState) {
            calibrationMode = false;
            resetRotationTracking();
        } else if (calibrationMode) {
            resetRotationTracking();
            calibrationJoystickInput = -gamepad.right_stick_x;
            calibrationJoystickInput = applyDeadzone(calibrationJoystickInput, 0.1);
        }
        lastLeftTriggerState = currentLeftTriggerState;
    }


    public void setTurretMode(TurretMode mode) {
        this.currentMode = mode;
        turretPID.reset();

        if (mode == TurretMode.FIELD_ANGLE) {
            fieldAngleTarget = getCurrentAngle();
        }
    }

    public TurretMode getTurretMode() {
        return currentMode;
    }

    public void setFieldAngleTarget(double fieldAngle) {
        this.fieldAngleTarget = normalizeAngle(fieldAngle);
    }


    public void setRobotRelativeAngle(double robotRelativeAngle) {
        this.targetTurretAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, robotRelativeAngle));
    }


    private void updateFieldTargetMode(Pose robotPose) {
        double fieldAngle = calculateFieldAngleToTarget(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        setTargetAngle(fieldAngle);
        updateFieldTargetMode();
    }

    private void updateFieldTargetMode() {
        double targetContinuous = turretToContinuousAngle(targetTurretAngle);
        double currentContinuous = getContinuousAngle();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetContinuous);

        double servoPower = turretPID.calculate(currentContinuous);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
        turretServo.setPower(-servoPower);
        turretServoLeft.setPower(-servoPower);
    }


    private void updateFieldAngleMode() {
        double robotHeading = Math.toDegrees(thisRobotPose.getHeading());
        double relativeTurretAngle = fieldAngleTarget - robotHeading;
        relativeTurretAngle = normalizeAngle(relativeTurretAngle);

        setTargetAngle(relativeTurretAngle);

        double targetContinuous = turretToContinuousAngle(targetTurretAngle);
        double currentContinuous = getContinuousAngle();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetContinuous);

        double servoPower = turretPID.calculate(currentContinuous);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
        turretServo.setPower(-servoPower);
        turretServoLeft.setPower(-servoPower);
    }


    private void updateRobotRelativeMode() {
        double targetContinuous = turretToContinuousAngle(targetTurretAngle);
        double currentContinuous = getContinuousAngle();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetContinuous);

        double servoPower = turretPID.calculate(currentContinuous);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
        turretServo.setPower(-servoPower);
        turretServoLeft.setPower(-servoPower);
    }


    public void setTargetAngle(double angle) {
        this.targetTurretAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, angle));
    }

    public void setManualPower(double power) {
        this.manualPower = power;
    }

    public void setAutoAimEnabled(boolean enabled) {
        this.autoAimEnabled = enabled;
        if (enabled) {
            turretPID.reset();
        }
    }

    public void setTargetPoint(double x, double y) {
        this.TARGET_X = x;
        this.TARGET_Y = y;
    }

    public void setCalibrationMode(boolean enabled, double joystickInput) {
        this.calibrationMode = enabled;
        this.calibrationJoystickInput = joystickInput;
        if (!enabled) {
            resetRotationTracking();
        }
    }

    public double getCurrentAngle() {
        return continuousToTurretAngle(getContinuousAngle());
    }

    public double getContinuousAngle() {
        return continuousAngle;
    }

    public double getRawAngle() {
        return encoder.getVoltage() * 360.0 / encoder.getMaxVoltage();
    }

    public void resetRotationTracking() {
        double currentRawAngle = getRawAngle();
        lastRawAngle = currentRawAngle;
        continuousAngle = currentRawAngle;
        lastContinuousAngle = continuousAngle;
        fullRotations = 0;
        firstUpdate = true;
    }

    public boolean isOnTarget(double tolerance) {
        double currentAngle = getCurrentAngle();
        return Math.abs(currentAngle - targetTurretAngle) < tolerance;
    }

    public double getTargetAngle() {
        return targetTurretAngle;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public boolean isCalibrationMode() {
        return calibrationMode;
    }

    private void updateManualToTarget() {
        double targetContinuous = turretToContinuousAngle(targetTurretAngle);
        double currentContinuous = getContinuousAngle();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetContinuous);

        double servoPower = turretPID.calculate(currentContinuous);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
        turretServo.setPower(-servoPower);
        turretServoLeft.setPower(-servoPower);
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

    private double calculateFieldAngleToTarget(double robotX, double robotY, double robotHeading) {
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        double absoluteAngleToTarget = Math.atan2(dy, dx);
        double relativeAngle = absoluteAngleToTarget - robotHeading;

        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

        return Math.toDegrees(relativeAngle);
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

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;
    }


    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public boolean isAtTargetAngle(double tolerance) {
        double currentAngle = getCurrentAngle();
        return Math.abs(currentAngle - targetTurretAngle) <= tolerance;
    }

    public boolean isAtTargetAngle() {
        return isAtTargetAngle(tolerance);
    }

    public boolean isAtAngle(double angle, double tolerance) {
        double currentAngle = getCurrentAngle();
        return Math.abs(currentAngle - angle) <= tolerance;
    }

    public boolean isInAngleRange(double minAngle, double maxAngle) {
        double currentAngle = getCurrentAngle();
        return currentAngle >= minAngle && currentAngle <= maxAngle;
    }

    public double getAngleError() {
        return targetTurretAngle - getCurrentAngle();
    }

    public double getAbsoluteAngleError() {
        return Math.abs(getAngleError());
    }


    public void showTelemetry(Telemetry telemetry) {
        double fieldAngle = calculateFieldAngleToTarget(thisRobotPose.getX(), thisRobotPose.getY(), thisRobotPose.getHeading());

        telemetry.addLine("=== TURRET CONTROLLER ===");
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", thisRobotPose.getX(), thisRobotPose.getY());
        telemetry.addData("Target Position", "X: %.1f, Y: %.1f", TARGET_X, TARGET_Y);
        telemetry.addData("Field Angle to Target", "%.1f°", fieldAngle);
        telemetry.addData("Turret Current Angle", "%.1f°", getCurrentAngle());
        telemetry.addData("Turret Target Angle", "%.1f°", targetTurretAngle);
        telemetry.addData("Current Mode", currentMode.toString());

        if (currentMode == TurretMode.FIELD_ANGLE) {
            telemetry.addData("Field Angle Target", "%.1f°", fieldAngleTarget);
        }

        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Calibration Mode", calibrationMode ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Continuous Angle", "%.1f°", getContinuousAngle());
        telemetry.addData("Raw Angle", "%.1f°", getRawAngle());
        telemetry.addData("Full Rotations", fullRotations);
    }
}