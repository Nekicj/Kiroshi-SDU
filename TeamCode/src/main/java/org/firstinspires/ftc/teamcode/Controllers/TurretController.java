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

    public static double TARGET_X = 119.4;
    public static double TARGET_Y = 88;

    private double lastRawAngle = 0;
    private int fullRotations = 0;
    private double continuousAngle = 0;
    private double lastContinuousAngle = 0;
    private boolean firstUpdate = true;
    private static final double MAX_REASONABLE_DELTA = 180;

    private boolean autoAimEnabled = true;
    private double targetTurretAngle = 0;
    private double manualPower = 0;

    private boolean calibrationMode = false;
    private double calibrationJoystickInput = 0;

    private Gamepad gamepad;
    private boolean lastLeftTriggerState = false;

    public void initialize(HardwareMap hardwareMap, String encoderName, String turretServoName) {
        turretPID = new asmPIDController(kP, kI, kD);
        turretPID.setTolerance(tolerance);

        encoder = hardwareMap.get(AnalogInput.class, encoderName);
        turretServo = hardwareMap.get(CRServo.class, turretServoName);

        lastRawAngle = getRawAngle();
        continuousAngle = lastRawAngle;
        lastContinuousAngle = continuousAngle;
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update(Pose robotPose) {
        if (!autoAimEnabled) {
            return;
        }
        updateContinuousAngle();
        processGamepadInput();

        if (calibrationMode) {
            turretServo.setPower(calibrationJoystickInput);
        } else if (autoAimEnabled) {
            updateAutoAim(robotPose);
        } else {
            updateManualToTarget();
        }
    }

    public void update() {
        updateContinuousAngle();
        processGamepadInput();

        if (calibrationMode) {
            turretServo.setPower(calibrationJoystickInput);
        } else if (autoAimEnabled) {
            updateAutoAim();
        } else {
            updateManualToTarget();
        }
    }

    private void processGamepadInput() {
        if (gamepad == null) return;

        boolean currentLeftTriggerState = gamepad.left_trigger > 0.1;
        if (currentLeftTriggerState && !lastLeftTriggerState) {
            calibrationMode = true;
            calibrationJoystickInput = -gamepad.right_stick_x;
        } else if (!currentLeftTriggerState && lastLeftTriggerState) {
            calibrationMode = false;
            resetRotationTracking();
        } else if (calibrationMode) {
            calibrationJoystickInput = -gamepad.right_stick_x;
            calibrationJoystickInput = applyDeadzone(calibrationJoystickInput, 0.1);
        }

        lastLeftTriggerState = currentLeftTriggerState;
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


    private void updateAutoAim(Pose robotPose) {

        double fieldAngle = calculateFieldAngleToTarget(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        setTargetAngle(fieldAngle);
        updateAutoAim();
    }

    private void updateAutoAim() {
        double targetContinuous = turretToContinuousAngle(targetTurretAngle);
        double currentContinuous = getContinuousAngle();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetContinuous);

        double servoPower = turretPID.calculate(currentContinuous);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));

        turretServo.setPower(-servoPower);
    }

    private void updateManualToTarget() {
        double targetContinuous = turretToContinuousAngle(targetTurretAngle);
        double currentContinuous = getContinuousAngle();

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetContinuous);

        double servoPower = turretPID.calculate(currentContinuous);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
        turretServo.setPower(-servoPower);
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


    public void showTelemetry(Telemetry telemetry, Pose robotPose) {
        double fieldAngle = calculateFieldAngleToTarget(robotPose.getX(), robotPose.getY(), robotPose.getHeading());

        telemetry.addLine("=== TURRET CONTROLLER ===");
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", robotPose.getX(), robotPose.getY());
        telemetry.addData("Target Position", "X: %.1f, Y: %.1f", TARGET_X, TARGET_Y);
        telemetry.addData("Field Angle to Target", "%.1f°", fieldAngle);
        telemetry.addData("Turret Current Angle", "%.1f°", getCurrentAngle());
        telemetry.addData("Turret Target Angle", "%.1f°", targetTurretAngle);
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Calibration Mode", calibrationMode ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Continuous Angle", "%.1f°", getContinuousAngle());
        telemetry.addData("Raw Angle", "%.1f°", getRawAngle());
        telemetry.addData("Full Rotations", fullRotations);

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Calibration", "Hold Left Trigger + Right Stick");
        telemetry.addData("Toggle Auto-Aim", "X button");
        telemetry.addData("Move Target (RB + DPad)", "Change target position");
        telemetry.addData("Manual Aim (no auto)", "DPad Up/Down");
    }
}