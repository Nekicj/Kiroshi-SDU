package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class asmCyclicPIDController {
    private double Kp, Ki, Kd;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double maxIntegralSum = 1.0;
    private double tolerance = 0;
    private double target;

    private boolean isCyclic = false;
    private double cycleRange = 360.0;

    public asmCyclicPIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        timer.reset();
    }
    public void setCyclic(boolean cyclic) {
        this.isCyclic = cyclic;
    }

    public void setCycleRange(double range) {
        this.cycleRange = range;
    }

    public void setTarget(double target) {
        this.target = target;
        this.integralSum = 0;
        this.lastError = 0;
    }

    public double calculate(double current) {
        double error;

        if (isCyclic) {
            error = calculateCyclicError(target, current);
        } else {
            error = target - current;
        }

        if (Math.abs(error) <= tolerance) {
            return 0;
        }

        double P = Kp * error;

        integralSum += error * timer.seconds();
        if (integralSum > maxIntegralSum) integralSum = maxIntegralSum;
        if (integralSum < -maxIntegralSum) integralSum = -maxIntegralSum;
        double I = Ki * integralSum;

        double derivative = (error - lastError) / timer.seconds();
        double D = Kd * derivative;

        lastError = error;
        timer.reset();

        return P + I + D;
    }

    private double calculateCyclicError(double target, double current) {
        double normalizedTarget = target % cycleRange;
        if (normalizedTarget < 0) normalizedTarget += cycleRange;

        double normalizedCurrent = current % cycleRange;
        if (normalizedCurrent < 0) normalizedCurrent += cycleRange;

        double error1 = normalizedTarget - normalizedCurrent;
        double error2 = error1 > 0 ? error1 - cycleRange : error1 + cycleRange;

        return Math.abs(error1) < Math.abs(error2) ? error1 : error2;
    }

    private double calculateCyclicErrorSimple(double target, double current) {
        double error = target - current;
        while (error > cycleRange / 2) {
            error -= cycleRange;
        }
        while (error < -cycleRange / 2) {
            error += cycleRange;
        }

        return error;
    }

    public void setTolerance(double newTolerance) {
        tolerance = newTolerance;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public void setCoefficients(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegralSum = maxIntegral;
    }

    public double getError() {
        return lastError;
    }

    public boolean atSetPoint() {
        return Math.abs(lastError) <= tolerance;
    }
}