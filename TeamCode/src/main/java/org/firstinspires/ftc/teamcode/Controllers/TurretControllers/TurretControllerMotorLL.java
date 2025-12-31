package org.firstinspires.ftc.teamcode.Controllers.TurretControllers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.asmPIDController;

@Config
public class TurretControllerMotorLL {
    private DcMotorEx turretMotor;
    private asmPIDController turretPID;
    private Limelight3A limelight;

    private Pose thisRobotPose = new Pose(0, 0, 0);

    public static double kP = 0.006;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 0.5;

    public static double MIN_TURRET_ANGLE = -180;
    public static double MAX_TURRET_ANGLE = 180;
    public static double ZERO_POSITION = 0;
    public static double MIN_POSITION = -596;
    public static double MAX_POSITION = 596;
    public static double TARGET_X = 21.71;
    public static double TARGET_Y = 45.22;

    public static double LIMELIGHT_CORRECTION_GAIN = 0.7;
    public static double LIMELIGHT_ANGLE_TOLERANCE = 0.2;
    public static int VALID_PIPELINE_ID = 0;

    public enum TurretMode {
        FIELD_TARGET,
        FIELD_ANGLE,
        ROBOT_RELATIVE,
        MANUAL
    }

    private TurretMode currentMode = TurretMode.FIELD_TARGET;
    private boolean autoAimEnabled = true;
    private double targetTurretAngle = 0;
    private double manualPower = 0;
    private double fieldAngleTarget = 0;


    private boolean limelightCorrectionEnabled = false;
    private boolean limelightValid = false;
    private double limelightTx = 0;
    private double limelightCorrectionAngle = 0;

    private Gamepad gamepad;
    private boolean calibrationMode = false;
    private double calibrationPower = 0;


    public void initialize(HardwareMap hardwareMap, String motorName) {
        initialize(hardwareMap, motorName, null);
    }

    public void initialize(HardwareMap hardwareMap, String motorName, String limelightName) {
        turretPID = new asmPIDController(kP, kI, kD);
        turretPID.setTolerance(1);

        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if (limelightName != null && !limelightName.isEmpty()) {
            try {
                limelight = hardwareMap.get(Limelight3A.class, limelightName);
                limelight.start();
            } catch (Exception e) {
                limelight = null;
            }
        }
    }


    public void update(Pose robotPose) {
        this.thisRobotPose = robotPose;

        updateLimelightData();

        if (gamepad != null && Math.abs(gamepad.right_stick_x) > 0.1) {
            manualPower = -gamepad.right_stick_x * 0.5;
            currentMode = TurretMode.MANUAL;
        } else if (currentMode == TurretMode.MANUAL) {
            manualPower = 0;
        }

        if (gamepad != null && gamepad.left_trigger > 0.5) {
            calibrationMode = true;
            if(gamepad.dpad_left){
                calibrationPower = -0.6;
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else if(gamepad.dpad_right){
                calibrationPower = 0.6;
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else{
                calibrationPower = 0.8;
            }
        } else {
            calibrationMode = false;
        }

        if (calibrationMode) {
            turretMotor.setPower(calibrationPower);
        } else if (currentMode == TurretMode.MANUAL) {
            turretMotor.setPower(manualPower);
        } else {
            switch (currentMode) {
                case FIELD_TARGET:
                    updateFieldTargetMode(robotPose);
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


    private void updateLimelightData() {
        limelightValid = false;
        limelightTx = 0;
        limelightCorrectionAngle = 0;

        if (limelight != null && limelightCorrectionEnabled) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                limelightValid = true;
                limelightTx = result.getTx();

                limelightCorrectionAngle = -limelightTx * LIMELIGHT_CORRECTION_GAIN;
            }
        }
    }



    private void updateFieldAngleMode() {
        if (limelightCorrectionEnabled && limelightValid) {
            double correction = -limelightTx * LIMELIGHT_CORRECTION_GAIN;

            setTargetAngle(correction);

            if (Math.abs(limelightTx) < LIMELIGHT_ANGLE_TOLERANCE) {
                turretPID.reset();
            }
        } else {
            double robotHeading = Math.toDegrees(thisRobotPose.getHeading());
            double relativeTurretAngle = fieldAngleTarget - robotHeading;
            relativeTurretAngle = normalizeAngle(relativeTurretAngle);
            setTargetAngle(relativeTurretAngle);
        }

        updatePID();
    }


    private void updateFieldTargetMode(Pose robotPose) {
        if (autoAimEnabled) {
            double fieldAngle = calculateFieldAngleToTarget(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
            setTargetAngle(fieldAngle);
        }
        updatePID();
    }

    private void updateRobotRelativeMode() {
        updatePID();
    }

    private void updatePID() {
        double targetPosition = angleToPosition(targetTurretAngle);
        double currentPosition = turretMotor.getCurrentPosition();

        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));

        turretPID.setTarget(targetPosition);
        double power = turretPID.calculate(currentPosition);

        power = Math.max(-1.0, Math.min(1.0, power));

        turretMotor.setPower(power);
    }


    public void update() {
        update(thisRobotPose);
    }

    private double calculateFieldAngleToTarget(double robotX, double robotY, double robotHeading) {
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        double absoluteAngleToTarget = Math.atan2(dy, dx);
        double relativeAngle = Math.toDegrees(absoluteAngleToTarget) - Math.toDegrees(robotHeading);

        return normalizeAngle(relativeAngle);
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }


    private double angleToPosition(double angle) {
        double normalizedAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, angle));
        double angleFromMin = normalizedAngle - MIN_TURRET_ANGLE;
        double angleRange = MAX_TURRET_ANGLE - MIN_TURRET_ANGLE;
        double positionRange = MAX_POSITION - MIN_POSITION;

        return MIN_POSITION + (angleFromMin / angleRange) * positionRange;
    }

    private double positionToAngle(double position) {
        double clampedPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
        double positionFromMin = clampedPosition - MIN_POSITION;
        double positionRange = MAX_POSITION - MIN_POSITION;
        double angleRange = MAX_TURRET_ANGLE - MIN_TURRET_ANGLE;

        return MIN_TURRET_ANGLE + (positionFromMin / positionRange) * angleRange;
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void setTurretMode(TurretMode mode) {
        this.currentMode = mode;
        turretPID.reset();
    }

    public void setPipeline(int pipelineId){
        limelight.pipelineSwitch(pipelineId);
    }

    public void setTargetAngle(double angle) {
        this.targetTurretAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, angle));
    }

    public void setFieldAngleTarget(double fieldAngle) {
        this.fieldAngleTarget = fieldAngle;
    }

    public void setRobotRelativeAngle(double angle) {
        this.targetTurretAngle = angle;
        this.currentMode = TurretMode.ROBOT_RELATIVE;
    }

    public void setAutoAimEnabled(boolean enabled) {
        this.autoAimEnabled = enabled;
    }

    public void setTargetPoint(double x, double y) {
        this.TARGET_X = x;
        this.TARGET_Y = y;
    }

    public void enableLimelightCorrection(boolean enabled) {
        this.limelightCorrectionEnabled = enabled;
    }

    public void toggleLimelightCorrection() {
        this.limelightCorrectionEnabled = !this.limelightCorrectionEnabled;
    }

    public boolean isLimelightCorrectionEnabled() {
        return limelightCorrectionEnabled;
    }

    public boolean isLimelightValid() {
        return limelightValid;
    }

    public double getLimelightTx() {
        return limelightTx;
    }

    // ==================== ГЕТТЕРЫ ====================

    public double getCurrentAngle() {
        return positionToAngle(turretMotor.getCurrentPosition());
    }

    public double getTargetAngle() {
        return targetTurretAngle;
    }

    public TurretMode getTurretMode() {
        return currentMode;
    }

    public boolean isOnTarget() {
        return Math.abs(getCurrentAngle() - targetTurretAngle) < tolerance;
    }



    public void showTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Current Angle", "%.1f°", getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", targetTurretAngle);
        telemetry.addData("Error", "%.1f°", targetTurretAngle - getCurrentAngle());
        telemetry.addData("Position", turretMotor.getCurrentPosition());
        telemetry.addData("Power", "%.2f", turretMotor.getPower());
        telemetry.addData("On Target", isOnTarget());

        telemetry.addLine("=== LIMELIGHT ===");
        telemetry.addData("Correction Enabled", limelightCorrectionEnabled ? "ON" : "OFF");
        telemetry.addData("Limelight Valid", limelightValid ? "YES" : "NO");
        if (limelightValid) {
            telemetry.addData("Tx", "%.2f°", limelightTx);
            telemetry.addData("Correction Angle", "%.2f°", limelightCorrectionAngle);
        }
    }
}