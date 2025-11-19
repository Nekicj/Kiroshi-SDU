package org.firstinspires.ftc.teamcode.Kotak.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils.asmPIDController;

@Config
@TeleOp(name = "Turret Servo PID", group = "Calibration")
public class TurretPIDKotak extends LinearOpMode {

    private AnalogInput encoder = null;
    private CRServo CrServo = null;
    private asmPIDController CRPID;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.001;
    public static double tolerance = 5;

    private boolean toTarget = false;

    private double lastRawAngle = 0;
    private int fullRotations = 0;
    private double continuousAngle = 0;

    public static double MIN_TURRET_ANGLE = -180;
    public static double MAX_TURRET_ANGLE = 180;
    public static double MIN_CONTINUOUS = -537;
    public static double MAX_CONTINUOUS = 700;
    public static double ZERO_CONTINUOUS = 40;

    public static double targetTurretAngle = 0;

    @Override
    public void runOpMode() {
        CRPID = new asmPIDController(kP, kI, kD);

        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        CrServo = hardwareMap.get(CRServo.class, "turret_r");

        lastRawAngle = getRawAngle();
        continuousAngle = lastRawAngle;

        waitForStart();

        while (opModeIsActive()) {
            updateContinuousAngle();

            double currentTurretAngle = continuousToTurretAngle(getContinuousAngle());

            CRPID.setCoefficients(kP, kI, kD);
            CRPID.setTolerance(tolerance);

            if (gamepad1.right_bumper) {
                toTarget = !toTarget;
                if (toTarget) {
                    CRPID.reset();
                }
                sleep(200);
            }

            double servoPower = 0;
            if (!toTarget) {
                if (gamepad1.y) {
                    CrServo.setPower(1);
                } else if (gamepad1.b) {
                    CrServo.setPower(-1);
                } else {
                    CrServo.setPower(0);
                }
            } else {
                double targetContinuous = turretToContinuousAngle(targetTurretAngle);
                CRPID.setTarget(targetContinuous);

                servoPower = CRPID.calculate(getContinuousAngle());
                servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
                CrServo.setPower(-servoPower);
            }

            updateTargetAngle();

            showTelemetry(currentTurretAngle);
        }
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


    private void updateTargetAngle() {
        if (gamepad1.dpad_up) {
            targetTurretAngle += 1;
            sleep(200);
        } else if (gamepad1.dpad_down) {
            targetTurretAngle -= 1;
        }
        if (gamepad1.dpad_right) {
            targetTurretAngle += 10;
            sleep(200);
        } else if (gamepad1.dpad_left) {
            targetTurretAngle -= 10;
            sleep(200);
        }

        targetTurretAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, targetTurretAngle));
    }


    private double getRawAngle() {
        return encoder.getVoltage() * 360.0 / encoder.getMaxVoltage();
    }

    private void updateContinuousAngle() {
        double currentRawAngle = getRawAngle();
        double delta = currentRawAngle - lastRawAngle;

        if (delta > 180) {
            fullRotations--;
        } else if (delta < -180) {
            fullRotations++;
        }

        lastRawAngle = currentRawAngle;
        continuousAngle = currentRawAngle + (fullRotations * 360.0);
    }

    private double getContinuousAngle() {
        return continuousAngle;
    }

    private void showTelemetry(double currentTurretAngle) {
        double currentContinuous = getContinuousAngle();
        double targetContinuous = turretToContinuousAngle(targetTurretAngle);

        telemetry.addLine("=== TURRET MANUAL CALIBRATION ===");
        telemetry.addData("Raw Angle", "%.1f°", getRawAngle());
        telemetry.addData("Continuous Angle", "%.1f°", currentContinuous);
        telemetry.addData("Turret Angle", "%.1f°", currentTurretAngle);
        telemetry.addData("Target Turret Angle", "%.1f°", targetTurretAngle);
        telemetry.addData("Target Continuous", "%.1f°", targetContinuous);
        telemetry.addData("Servo Power", "%.3f", -CrServo.getPower());

        telemetry.addLine("");
        telemetry.addLine("=== CALIBRATION PARAMETERS (Set in Dashboard) ===");
        telemetry.addData("MIN_TURRET_ANGLE", "%.1f°", MIN_TURRET_ANGLE);
        telemetry.addData("MAX_TURRET_ANGLE", "%.1f°", MAX_TURRET_ANGLE);
        telemetry.addData("MIN_CONTINUOUS", "%.1f", MIN_CONTINUOUS);
        telemetry.addData("MAX_CONTINUOUS", "%.1f", MAX_CONTINUOUS);
        telemetry.addData("ZERO_CONTINUOUS", "%.1f", ZERO_CONTINUOUS);

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Toggle Auto/Manual", "Right Bumper");
        telemetry.addData("Manual Control", "Y/B buttons");
        telemetry.addData("Adjust Target Angle", "DPad (Up/Down: ±1°, Left/Right: ±10°)");
        telemetry.addData("Current Mode", toTarget ? "AUTO" : "MANUAL");
        telemetry.addData("Full Rotations", fullRotations);

        telemetry.addLine("");
        telemetry.addLine("=== CALIBRATION CHECK ===");
        telemetry.addData("Calculated -90° continuous", "%.1f", turretToContinuousAngle(-90));
        telemetry.addData("Calculated 0° continuous", "%.1f", turretToContinuousAngle(0));
        telemetry.addData("Calculated 90° continuous", "%.1f", turretToContinuousAngle(90));

        telemetry.update();
    }
}