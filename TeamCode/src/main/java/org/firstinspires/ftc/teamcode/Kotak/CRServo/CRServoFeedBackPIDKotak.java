package org.firstinspires.ftc.teamcode.Kotak.CRServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils.asmPIDController;

@Config
@TeleOp(name = "Axon CRServo PID ", group = "nigga")
public class CRServoFeedBackPIDKotak extends LinearOpMode {

    private AnalogInput encoder = null;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.001;

    public static double target = 0;
    public static double tolerance = 5;

    private boolean toTarget = false;
    private asmPIDController CRPID;
    private CRServo CrServo = null;

    private double lastRawAngle = 0;
    private int fullRotations = 0;
    private double continuousAngle = 0;

    public void runOpMode() {
        CRPID = new asmPIDController(kP, kI, kD);
        CRPID.setTarget(target);
        CRPID.setTolerance(tolerance);

        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        CrServo = hardwareMap.get(CRServo.class, "turret_r");

        lastRawAngle = getRawAngle();

        waitForStart();

        while (opModeIsActive()) {
            CRPID.setCoefficients(kP, kI, kD);
            CRPID.setTarget(target);
            CRPID.setTolerance(tolerance);

            double currentAngle = getContinuousAngle();
            double displayAngle = currentAngle % 360;
            if (gamepad1.rightBumperWasPressed()) {
                toTarget = !toTarget;
                if (toTarget) {
                    CRPID.reset();
                }
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
            }
            else {
                servoPower = CRPID.calculate(currentAngle);

//                servoPower = checkTolerance(servoPower, 0.05);
                servoPower = Math.max(-1.0, Math.min(1.0, servoPower));

                CrServo.setPower(-servoPower);
            }

            telemetry.addData("Raw Angle", "%.1f°", getRawAngle());
            telemetry.addData("Continuous Angle", "%.1f°", currentAngle);
            telemetry.addData("Display Angle", "%.1f°", displayAngle);
            telemetry.addData("Target Angle", "%.1f°", target);
            telemetry.addData("Servo Power", "%.3f", servoPower);
            telemetry.addData("Mode", toTarget ? "AUTO" : "MANUAL");
            telemetry.addData("Full Rotations", fullRotations);
            telemetry.update();
        }
    }

    private double getRawAngle() {
        return encoder.getVoltage() * 360.0 / encoder.getMaxVoltage();
    }


    private double getContinuousAngle() {
        double currentRawAngle = getRawAngle();

        double delta = currentRawAngle - lastRawAngle;

        if (delta > 180) {
            fullRotations--;
        } else if (delta < -180) {
            fullRotations++;
        }

        lastRawAngle = currentRawAngle;

        continuousAngle = currentRawAngle + (fullRotations * 360.0);
        return continuousAngle;
    }


    private double checkTolerance(double value, double tolerance) {
        if (Math.abs(value) < tolerance) {
            return 0;
        }
        return value;
    }

    private void resetRotationTracking() {
        fullRotations = 0;
        lastRawAngle = getRawAngle();
        continuousAngle = lastRawAngle;
    }
}