package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class IntakeController {
    private DcMotor intakeMotor = null;
    private DcMotor secIntakeMotor = null;

    public void initialize(HardwareMap hardwareMap,String intakeMotorName,String secIntakeMotorName){
        intakeMotor = hardwareMap.get(DcMotor.class, intakeMotorName);
        secIntakeMotor = hardwareMap.get(DcMotor.class,secIntakeMotorName);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        secIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setIntakePower(double intakePower){
        intakeMotor.setPower(intakePower);
    }
    public void setSecIntakeMotor(double power){
        secIntakeMotor.setPower(power);
    }
}
