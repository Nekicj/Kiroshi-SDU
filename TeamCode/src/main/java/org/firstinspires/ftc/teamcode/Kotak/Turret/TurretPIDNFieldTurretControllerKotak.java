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
@TeleOp(name = "Turret Field Auto-Aim", group = "Competition")
public class TurretPIDNFieldTurretControllerKotak extends LinearOpMode {

    private Follower follower;
    private asmGamepadEx driver1;

    private AnalogInput encoder;
    private CRServo turretServo;
    private asmPIDController turretPID;

    // PID параметры
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.001;
    public static double tolerance = 5;

    // Калибровочные параметры турели (из вашего работающего кода)
    public static double MIN_TURRET_ANGLE = -90;
    public static double MAX_TURRET_ANGLE = 90;
    public static double MIN_CONTINUOUS = -412;
    public static double MAX_CONTINUOUS = 490;
    public static double ZERO_CONTINUOUS = 40;

    // Целевая точка на поле (в см)
    public static double TARGET_X = 100;
    public static double TARGET_Y = 100;

    // Переменные для отслеживания continuous angle
    private double lastRawAngle = 0;
    private int fullRotations = 0;
    private double continuousAngle = 0;

    // Режимы работы
    private boolean autoAimEnabled = true;
    private boolean manualOverride = false;
    private boolean toTarget = false;

    // Текущий целевой угол турели (для ручного управления)
    public static double targetTurretAngle = 0;

    @Override
    public void runOpMode() {
        // Инициализация системы движения
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Инициализация турели
        initializeTurret();

        driver1 = new asmGamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            driver1.update();

            // Управление базой робота
            driveBase();

            // Обновление continuous angle
            updateContinuousAngle();

            // Автонаведение или ручное управление
            if (autoAimEnabled && !manualOverride) {
                updateAutoAim();
            } else {
                updateManualTurretControl();
            }

            // Переключение режимов
            updateModeSwitching();

            // Телеметрия
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
    }

    private void driveBase() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        // Применяем deadzone
        forward = applyDeadzone(forward, 0.05);
        strafe = applyDeadzone(strafe, 0.05);
        rotate = applyDeadzone(rotate, 0.05);

        follower.update();
        follower.setTeleOpDrive(forward, strafe, rotate * 0.7, false);
    }

    private void updateAutoAim() {
        // Получаем позицию и ориентацию робота
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading(); // в радианах

        // Вычисляем угол к цели на поле
        double fieldAngleToTarget = calculateFieldAngleToTarget(robotX, robotY, robotHeading);

        // Преобразуем в угол турели и ограничиваем диапазоном
        double turretTargetAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, fieldAngleToTarget));

        // Преобразуем в continuous angle для PID
        double targetContinuous = turretToContinuousAngle(turretTargetAngle);

        // Получаем текущий continuous angle
        double currentContinuous = getContinuousAngle();

        // Вычисляем мощность через PID
        turretPID.setCoefficients(kP, kI, kD);
        turretPID.setTarget(targetContinuous);

        double servoPower = turretPID.calculate(currentContinuous);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));

        // Применяем мощность к турели
        turretServo.setPower(-servoPower);
    }

    private void updateManualTurretControl() {
        if (!toTarget) {
            // Прямое ручное управление сервой
            if (gamepad1.y) {
                turretServo.setPower(1);
            } else if (gamepad1.b) {
                turretServo.setPower(-1);
            } else {
                turretServo.setPower(0);
            }
        } else {
            // Управление к конкретному углу через PID
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
        // Переключение автонаведения (X button)
        if (gamepad1.x) {
            autoAimEnabled = !autoAimEnabled;
            sleep(200);
        }

        // Переключение ручного управления турелью (Right Bumper)
        if (gamepad1.right_bumper) {
            toTarget = !toTarget;
            if (toTarget) {
                turretPID.reset();
            }
            sleep(200);
        }

        // Временное ручное управление (Left Bumper)
        if (gamepad1.left_bumper) {
            manualOverride = true;
        } else {
            manualOverride = false;
        }

        // Изменение целевой точки на поле (DPad)
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

        // Изменение целевого угла в ручном режиме (для toTarget)
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
    }

    /**
     * Вычисляет угол от робота к цели на поле с учетом ориентации робота
     * Возвращает угол в градусах для турели (-90 до 90)
     */
    private double calculateFieldAngleToTarget(double robotX, double robotY, double robotHeading) {
        // Вектор от робота к цели
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        // Абсолютный угол к цели на поле (в радианах)
        double absoluteAngleToTarget = Math.atan2(dy, dx);

        // Относительный угол с учетом направления робота
        double relativeAngle = absoluteAngleToTarget - robotHeading;

        // Нормализуем угол в диапазон [-PI, PI]
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

        // Конвертируем в градусы
        double angleDegrees = Math.toDegrees(relativeAngle);

        return angleDegrees;
    }

    // ========== МЕТОДЫ ПРЕОБРАЗОВАНИЯ УГЛОВ (ИЗ ВАШЕГО РАБОЧЕГО КОДА) ==========

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

    // ========== МЕТОДЫ ДЛЯ РАБОТЫ С ЭНКОДЕРОМ ==========

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

        telemetry.addLine("=== TURRET FIELD AUTO-AIM ===");
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", robotPose.getX(), robotPose.getY());
        telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Target Position", "X: %.1f, Y: %.1f", TARGET_X, TARGET_Y);
        telemetry.addData("Field Angle to Target", "%.1f°", fieldAngle);
        telemetry.addData("Limited Field Angle", "%.1f°", limitedFieldAngle);
        telemetry.addData("Turret Current Angle", "%.1f°", currentTurretAngle);
        telemetry.addData("Continuous Angle", "%.1f°", currentContinuous);

        telemetry.addLine("");
        telemetry.addLine("=== MODES ===");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Manual Override", manualOverride ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Turret Control", toTarget ? "ANGLE TARGET" : "DIRECT MANUAL");

        if (!autoAimEnabled && toTarget) {
            telemetry.addData("Manual Target Angle", "%.1f°", targetTurretAngle);
        }

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Toggle Auto-Aim", "X button");
        telemetry.addData("Toggle Turret Control", "Right Bumper");
        telemetry.addData("Temporary Manual", "Hold Left Bumper");
        telemetry.addData("Move Field Target", "DPad");

        if (!autoAimEnabled && toTarget) {
            telemetry.addData("Adjust Manual Target", "Y/B buttons");
        } else if (!autoAimEnabled) {
            telemetry.addData("Direct Manual Control", "Y/B buttons");
        }

        telemetry.update();
    }
}