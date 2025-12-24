package org.firstinspires.ftc.teamcode.Controllers.TurretControllers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.asmPIDController;

@Config
public class TurretControllerMotor {
    private DcMotorEx turretMotor;
    private asmPIDController turretPID;

    private Pose thisRobotPose = new Pose(0, 0, 0);

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 5;

    public static double MIN_TURRET_ANGLE = -180;
    public static double MAX_TURRET_ANGLE = 180;

    public static double MIN_POSITION = -5000;
    public static double MAX_POSITION = 5000;


    public static double POSITIONS_PER_DEGREE = 0;

    public static double TARGET_X = -8.87;
    public static double TARGET_Y = 33.91;

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

    public void initialize(HardwareMap hardwareMap, String turretMotorName) {
        turretPID = new asmPIDController(kP, kI, kD);
        turretPID.setTolerance(tolerance);

        turretMotor = hardwareMap.get(DcMotorEx.class, turretMotorName);

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        updatePositionsPerDegree();
    }
    public void updatePositionsPerDegree() {
        if (MAX_POSITION != 0 && MIN_POSITION != 0) {
            // (MAX_POSITION - MIN_POSITION) / 360 = тиков на градус
            POSITIONS_PER_DEGREE = (MAX_POSITION - MIN_POSITION) / 360.0;
        }
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update(Pose robotPose) {
        this.thisRobotPose = robotPose;
        updatePositionsPerDegree();
        processCalibrationInput();

        if (calibrationMode) {
            turretMotor.setPower(calibrationJoystickInput);
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
        updatePositionsPerDegree();
        processCalibrationInput();

        if (calibrationMode) {
            turretMotor.setPower(calibrationJoystickInput);
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
        } else if (!currentLeftTriggerState && lastLeftTriggerState) {
            calibrationMode = false;
        } else if (calibrationMode) {
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
        double targetPosition = angleToPosition(targetTurretAngle);
        double currentPosition = getCurrentPosition();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetPosition);

        double motorPower = turretPID.calculate(currentPosition);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        turretMotor.setPower(motorPower);
    }

    private void updateFieldAngleMode() {
        double robotHeading = Math.toDegrees(thisRobotPose.getHeading());
        double relativeTurretAngle = fieldAngleTarget - robotHeading;
        relativeTurretAngle = normalizeAngle(relativeTurretAngle);

        setTargetAngle(relativeTurretAngle);

        double targetPosition = angleToPosition(targetTurretAngle);
        double currentPosition = getCurrentPosition();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetPosition);

        double motorPower = turretPID.calculate(currentPosition);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
        turretMotor.setPower(motorPower);
    }

    private void updateRobotRelativeMode() {
        double targetPosition = angleToPosition(targetTurretAngle);
        double currentPosition = getCurrentPosition();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetPosition);

        double motorPower = turretPID.calculate(currentPosition);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
        turretMotor.setPower(motorPower);
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
    }

    public double getCurrentAngle() {
        return positionToAngle(getCurrentPosition());
    }

    public double getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void resetPosition() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        targetTurretAngle = 0;
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
        double targetPosition = angleToPosition(targetTurretAngle);
        double currentPosition = getCurrentPosition();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetPosition);

        double motorPower = turretPID.calculate(currentPosition);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
        turretMotor.setPower(motorPower);
    }

    private double angleToPosition(double angle) {
        return angle * POSITIONS_PER_DEGREE;
    }

    private double positionToAngle(double position) {
        if (POSITIONS_PER_DEGREE == 0) {
            return 0;
        }
        return position / POSITIONS_PER_DEGREE;
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
        telemetry.addData("Turret Current Position", "%.0f", getCurrentPosition());
        telemetry.addData("POSITIONS_PER_DEGREE", "%.3f", POSITIONS_PER_DEGREE);
        telemetry.addData("MIN/MAX Position", "%.0f / %.0f", MIN_POSITION, MAX_POSITION);
        telemetry.addData("Current Mode", currentMode.toString());

        if (currentMode == TurretMode.FIELD_ANGLE) {
            telemetry.addData("Field Angle Target", "%.1f°", fieldAngleTarget);
        }

        telemetry.addData("AutoAim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Calibration Mode", calibrationMode ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Motor Power", "%.3f", turretMotor.getPower());
    }
}

/**
 * я не знаю твое имя
 * но ты в моей голове
 * я хожу по универу
 * они смотрят на мой смех
 *
 * ваши правила ничто и я курю на этаже
 * плевать на ваше мнение
 * я живу уже в москве
 */