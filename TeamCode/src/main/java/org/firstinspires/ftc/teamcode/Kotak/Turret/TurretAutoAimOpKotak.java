package org.firstinspires.ftc.teamcode.Kotak.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.Utils.asmPIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Turret Auto Aim FIXED", group = "Competition")
public class TurretAutoAimOpKotak extends LinearOpMode {

    public static double TARGET_X = -10;
    public static double TARGET_Y = 10;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 5;

    // Физические ограничения турели (из измерений)
    public static double TURRET_MIN_ANGLE = -500;
    public static double TURRET_MAX_ANGLE = 500;

    private Follower follower;
    private asmGamepadEx driver1;

    private AnalogInput encoder;
    private CRServo turretServo;
    private asmPIDController turretPID;

    // Переменные для отслеживания непрерывного угла
    private double lastRawAngle = 0;
    private int fullRotations = 0;
    private double continuousAngle = 0;

    private boolean autoAimEnabled = true;
    private boolean manualOverride = false;

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

            // Всегда обновляем непрерывный угол
            updateContinuousAngle();

            if (autoAimEnabled && !manualOverride) {
                updateAutoAim();
            }
            updateManualTurretControl();
            updateModeSwitching();
            showTelemetry();
        }
    }

    private void initializeTurret() {
        turretPID = new asmPIDController(kP, kI, kD);
        turretPID.setTolerance(tolerance);

        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        turretServo = hardwareMap.get(CRServo.class, "turret_r");

        // Инициализация отслеживания угла
        lastRawAngle = getRawAngle();
        continuousAngle = lastRawAngle;
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

        // Вычисляем угол к цели в радианах
        double targetAngleRad = calculateAngleToTarget(robotX, robotY, robotHeading);

        // Преобразуем в градусы
        double targetAngleDeg = Math.toDegrees(targetAngleRad);

        // Нормализуем угол к цели в диапазон [-180, 180]
        targetAngleDeg = normalizeAngle(targetAngleDeg);

        // Преобразуем в систему координат турели (0° = влево, 180° = вправо)
        double turretTargetAngle = 90 + targetAngleDeg;

        // Ограничиваем физическими пределами турели [0, 180]
        turretTargetAngle = Math.max(0, Math.min(180, turretTargetAngle));

        // Получаем текущий continuous угол
        double currentContinuousAngle = getContinuousAngle();

        // Преобразуем target angle в continuous систему
        double continuousTarget = convertToContinuousTarget(turretTargetAngle, currentContinuousAngle);

        // Ограничиваем continuous target физическими пределами
        continuousTarget = Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, continuousTarget));

        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(continuousTarget);

        double servoPower = turretPID.calculate(currentContinuousAngle);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));

        turretServo.setPower(-servoPower);
    }

    /**
     * Преобразует целевой угол турели (0-180) в continuous target с учетом текущего положения
     * Выбирает ближайший эквивалентный угол в continuous системе
     */
    private double convertToContinuousTarget(double targetAngle, double currentContinuousAngle) {
        // Основной target в continuous системе (предполагая, что 0° continuous = 0° турели)
        double baseTarget = targetAngle;

        // Находим ближайший эквивалентный угол в continuous системе
        // Учитываем, что физический угол повторяется каждые 360 градусов
        double[] possibleTargets = {
                baseTarget - 360,
                baseTarget,
                baseTarget + 360
        };

        // Выбираем target, который ближе всего к текущему положению
        double bestTarget = baseTarget;
        double minDistance = Math.abs(currentContinuousAngle - baseTarget);

        for (double possibleTarget : possibleTargets) {
            double distance = Math.abs(currentContinuousAngle - possibleTarget);
            if (distance < minDistance &&
                    possibleTarget >= TURRET_MIN_ANGLE &&
                    possibleTarget <= TURRET_MAX_ANGLE) {
                minDistance = distance;
                bestTarget = possibleTarget;
            }
        }

        return bestTarget;
    }

    private void updateManualTurretControl() {
        if (manualOverride) {
            double servoPower = 0;
            if (gamepad1.y) {
                servoPower = 1;
            } else if (gamepad1.b) {
                servoPower = -1;
            }
            turretServo.setPower(servoPower);
        }
    }

    private void updateModeSwitching() {
        if (gamepad1.x) {
            autoAimEnabled = !autoAimEnabled;
            if (autoAimEnabled) {
                turretPID.reset();
            }
            sleep(200);
        }

        if (gamepad1.right_bumper) {
            manualOverride = true;
        } else {
            manualOverride = false;
        }

        if (gamepad1.dpad_up) {
            TARGET_Y += 10;
        } else if (gamepad1.dpad_down) {
            TARGET_Y -= 10;
        }
        if (gamepad1.dpad_right) {
            TARGET_X += 10;
        } else if (gamepad1.dpad_left) {
            TARGET_X -= 10;
        }
    }

    /**
     * Вычисляет угол от робота к цели в радианах
     */
    private double calculateAngleToTarget(double robotX, double robotY, double robotHeading) {
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        // Абсолютный угол к цели на поле
        double absoluteAngleToTarget = Math.atan2(dy, dx);

        // Относительный угол с учетом направления робота
        double relativeAngle = absoluteAngleToTarget - robotHeading;

        // Нормализуем в диапазон [-PI, PI]
        return normalizeAngleRad(relativeAngle);
    }

    /**
     * Нормализует угол в радианах в диапазон [-PI, PI]
     */
    private double normalizeAngleRad(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Нормализует угол в градусах в диапазон [-180, 180]
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // ========== МЕТОДЫ ДЛЯ РАБОТЫ С ЭНКОДЕРОМ ==========

    private double getRawAngle() {
        return encoder.getVoltage() * 360.0 / encoder.getMaxVoltage();
    }

    private void updateContinuousAngle() {
        double currentRawAngle = getRawAngle();
        double delta = currentRawAngle - lastRawAngle;

        // Обработка перехода через 0/360
        if (delta > 180) {
            fullRotations--;
        } else if (delta < -180) {
            fullRotations++;
        }

        lastRawAngle = currentRawAngle;
        continuousAngle = currentRawAngle + (fullRotations * 360.0);

        // Ограничиваем continuous angle физическими пределами (для безопасности)
        if (continuousAngle < TURRET_MIN_ANGLE) {
            continuousAngle = TURRET_MIN_ANGLE;
            // Сбрасываем отслеживание, чтобы избежать накопления ошибки
            resetRotationTracking();
        } else if (continuousAngle > TURRET_MAX_ANGLE) {
            continuousAngle = TURRET_MAX_ANGLE;
            resetRotationTracking();
        }
    }

    private double getContinuousAngle() {
        return continuousAngle;
    }

    /**
     * Сбрасывает отслеживание вращения (при достижении физических пределов)
     */
    private void resetRotationTracking() {
        fullRotations = 0;
        lastRawAngle = getRawAngle();
        continuousAngle = lastRawAngle;
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;
    }

    private void showTelemetry() {
        Pose robotPose = follower.getPose();
        double currentContinuousAngle = getContinuousAngle();
        double currentDisplayAngle = currentContinuousAngle % 360;
        double targetAngleRad = calculateAngleToTarget(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        double targetAngleDeg = Math.toDegrees(targetAngleRad);
        double turretTargetAngle = 90 + normalizeAngle(targetAngleDeg);
        turretTargetAngle = Math.max(0, Math.min(180, turretTargetAngle));
        double continuousTarget = convertToContinuousTarget(turretTargetAngle, currentContinuousAngle);

        telemetry.addLine("========== Turret Auto-Aim FIXED ==========");
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", robotPose.getX(), robotPose.getY());
        telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Target Position", "X: %.1f, Y: %.1f", TARGET_X, TARGET_Y);
        telemetry.addData("Angle to Target", "%.1f°", targetAngleDeg);
        telemetry.addData("Turret Target", "%.1f°", turretTargetAngle);
        telemetry.addData("Continuous Target", "%.1f°", continuousTarget);
        telemetry.addData("Turret Current", "%.1f° (Raw: %.1f°)", currentDisplayAngle, getRawAngle());
        telemetry.addData("Continuous Angle", "%.1f°", currentContinuousAngle);
        telemetry.addData("Full Rotations", fullRotations);
        telemetry.addData("Turret Limits", "%.0f° to %.0f°", TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);

        // Показываем предупреждение если接近 пределам
        if (Math.abs(currentContinuousAngle - TURRET_MIN_ANGLE) < 50 ||
                Math.abs(currentContinuousAngle - TURRET_MAX_ANGLE) < 50) {
            telemetry.addLine("*** WARNING: Near physical limit! ***");
        }

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Manual Override", manualOverride ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Toggle Auto-Aim", "X button");
        telemetry.addData("Manual Control", "Right Bumper + Y/B");
        telemetry.addData("Move Target", "DPad");
        telemetry.update();
    }
}