package org.firstinspires.ftc.teamcode.Kotak.TestOpModes.PartsTest;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;


@Config
@Disabled
@TeleOp(name = "ShooterTest",group = "Test")
public class ShooterTest extends LinearOpMode {
    private asmGamepadEx driver1;

    private double count = 0;

    ElapsedTime elapsedTimer = new ElapsedTime();

    private DcMotor shooterMotorLeft = null;
    private DcMotor shooterMotorRight = null;

    private boolean isShootint = false;
    public static double shooterSpeed = -1;

    @Override
    public void runOpMode() {
        shooterMotorLeft = hardwareMap.get(DcMotor.class,"shooterMotorLeft");
        shooterMotorRight = hardwareMap.get(DcMotor.class,"shooterMotorRight");

        shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        driver1 = new asmGamepadEx(gamepad1);


        telemetry.addData("Status, ","Initialized");
        elapsedTimer.reset();
        waitForStart();

        while (opModeIsActive()){
            driver1.update();

            if(driver1.isBPressed()){
                isShootint = !isShootint;
            }

            if(isShootint){
                shooterMotorRight.setPower(shooterSpeed);
                shooterMotorLeft.setPower(shooterSpeed);
            }else{
                shooterMotorRight.setPower(0);
                shooterMotorLeft.setPower(0);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("count",count);
             telemetry.update();

        }
    }

}
