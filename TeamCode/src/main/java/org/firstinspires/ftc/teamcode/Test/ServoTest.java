package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Config
@TeleOp(name="servo standard pos",group = "Test")
public class ServoTest extends LinearOpMode {

    private Servo servol = null;
    private Servo servor = null;

    public static double servoPos = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        servol = hardwareMap.get(Servo.class,"l_angle");
        servor = hardwareMap.get(Servo.class,"r_angle");
        servor.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.y){
                servol.setPosition(servoPos);
                servor.setPosition(servoPos);
            }
        }

    }
}
