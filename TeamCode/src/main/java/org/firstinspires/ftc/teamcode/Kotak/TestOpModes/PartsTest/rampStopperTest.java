package org.firstinspires.ftc.teamcode.Kotak.TestOpModes.PartsTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Ramp stopper test",group = "Test")
public class rampStopperTest extends LinearOpMode {

    private Servo ramp = null;
    private Servo stopper = null;
    private Servo r_angle = null;

    public static double servoramp = 0.2;
    public static double servostopper= 0.4;
    public static double servoAngle = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        ramp = hardwareMap.get(Servo.class,"ramp");
        stopper = hardwareMap.get(Servo.class,"stopper");
        r_angle = hardwareMap.get(Servo.class,"r_angle");


        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.y){
                ramp.setPosition(servoramp);
            }

            if(gamepad1.b){
                stopper.setPosition(servostopper);
            }

            if (gamepad1.a) {
                r_angle.setPosition(servoAngle);
            }
        }

    }
}
