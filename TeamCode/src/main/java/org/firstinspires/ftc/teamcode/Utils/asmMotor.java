package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class asmMotor {
    public enum GoBildaType {
        RPM_30(5264, 30),
        RPM_43(3892, 43),
        RPM_60(2786, 60),
        RPM_84(1993.6, 84),
        RPM_117(1425.2, 117),
        RPM_223(753.2, 223),
        RPM_312(537.6, 312),
        RPM_435(383.6, 435),
        RPM_1150(145.6, 1150),
        RPM_1620(103.6, 1620);

        private final double cpr;
        private final double rpm;

        GoBildaType(double cpr, double rpm) {
            this.cpr = cpr;
            this.rpm = rpm;

        }

        public double getCPR() {
            return cpr;
        }

        public double getRPM() {
            return rpm;
        }
    }

    private final DcMotorEx motor;
    private final GoBildaType type;
    private final double maxTicksPerSecond;

    private double targetVelocity = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastPosition = 0;
    private double currentVelocity = 0;

    public asmMotor(HardwareMap hardwareMap, String motorName, GoBildaType motorType) {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.type = motorType;
        this.maxTicksPerSecond = (motorType.getRPM() * motorType.getCPR()) / 60.0;

        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        timer.reset();
        lastPosition = motor.getCurrentPosition();
    }

    public void setVelocityRPM(double rpm) {
        double ticksPerSecond = (rpm * type.getCPR()) / 60.0;
        setVelocityTicksPerSecond(ticksPerSecond);
    }

    public void setVelocityTicksPerSecond(double ticksPerSecond) {
        this.targetVelocity = ticksPerSecond;
        double normalizedPower = ticksPerSecond / maxTicksPerSecond;
        motor.setPower(normalizedPower);
    }

    public void setPower(double power) {
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
        targetVelocity = 0;
    }

    public void update() {
        double currentTime = timer.seconds();
        double currentPosition = motor.getCurrentPosition();

        if (currentTime > 0) {
            currentVelocity = (currentPosition - lastPosition) / currentTime;
        }

        timer.reset();
        lastPosition = currentPosition;
    }

    public double getCurrentVelocityRPM() {
        return (currentVelocity * 60.0) / type.getCPR();
    }

    public double getCurrentVelocityTicksPerSecond() {
        return currentVelocity;
    }

    public double getTargetVelocityRPM() {
        return (targetVelocity * 60.0) / type.getCPR();
    }

    public double getTargetVelocityTicksPerSecond() {
        return targetVelocity;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getCPR() {
        return type.getCPR();
    }

    public double getMaxRPM() {
        return type.getRPM();
    }

    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
    }

    public void setInverted(boolean inverted) {
        motor.setDirection(inverted ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
    }

    public void resetEncoder() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lastPosition = 0;
    }
}