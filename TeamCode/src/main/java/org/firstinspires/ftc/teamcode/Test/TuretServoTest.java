package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Disabled
@TeleOp(name = "синдром туретта",group = "compet")
public class TuretServoTest extends LinearOpMode {

    private Servo leftServo,rightServo;

    public static double servoPose = 0.5;

    @Override
    public void runOpMode(){
        leftServo = hardwareMap.get(Servo.class,"turret_r");

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                leftServo.setPosition(1);
            }else if(gamepad1.b){
                leftServo.setPosition(0);
            }else{
                leftServo.setPosition(servoPose);
            }

            telemetry.addData("радмир","гандон");

        }
    }
}
