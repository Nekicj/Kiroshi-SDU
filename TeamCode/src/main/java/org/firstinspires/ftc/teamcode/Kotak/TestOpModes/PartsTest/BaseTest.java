package org.firstinspires.ftc.teamcode.Kotak.TestOpModes.PartsTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Base test",group = "Test")
public class BaseTest extends LinearOpMode {

    DcMotor Lfront = null;
    DcMotor Rfront = null;
    DcMotor Rback = null;
    DcMotor Lback = null;


    @Override
    public void runOpMode(){

        Lfront = hardwareMap.get(DcMotor.class,"lfd");
        Rfront = hardwareMap.get(DcMotor.class,"rfd");
        Lback = hardwareMap.get(DcMotor.class,"lbd");
        Rback = hardwareMap.get(DcMotor.class,"rbd");

        Lfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Rfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Lback.setDirection(DcMotorSimple.Direction.FORWARD);
        Rback.setDirection(DcMotorSimple.Direction.FORWARD);

        Lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.rightBumperWasReleased()){
                Lfront.setPower(1);
            }else{
                Lfront.setPower(0);
            }

            if(gamepad1.leftBumperWasReleased()){
                Rfront.setPower(1);
            }else{
                Rfront.setPower(0);
            }

            if(gamepad2.rightBumperWasReleased()){
                Lback.setPower(1);
            }else{
                Lback.setPower(0);
            }

            if(gamepad2.leftBumperWasReleased()){
                Rback.setPower(1);
            }else{
                Rback.setPower(0);
            }




        }
    }
}
