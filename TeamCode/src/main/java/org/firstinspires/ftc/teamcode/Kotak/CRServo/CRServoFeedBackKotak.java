package org.firstinspires.ftc.teamcode.Kotak.CRServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils.asmPIDController;

@Config
@Disabled
@TeleOp(name = "Servo test",group = "nigga")
public class CRServoFeedBackKotak extends LinearOpMode {

    private AnalogInput encoder = null;

    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0;

    public static double target = 180;
    public static double tolerance = 0;

    private double position = 0;

    private boolean toTarget = false;

    private asmPIDController CRPID = new asmPIDController(kP,kI,kD);

    private CRServo CrServo = null;
    private double maxPose = 0;

    public void runOpMode(){
        CRPID.setTarget(target);
        CRPID.setTolerance(tolerance);

        encoder = hardwareMap.get(AnalogInput.class,"encoder");
        CrServo = hardwareMap.get(CRServo.class,"turret_r");

        waitForStart();
        while(opModeIsActive()){
            CRPID.setTarget(target);
            CRPID.setCoefficients(kP,kI,kD);

            double servoPosition = encoder.getVoltage() * 355 /encoder.getMaxVoltage();


            if(maxPose<servoPosition){
                maxPose = servoPosition;
            }

            if(gamepad1.rightBumperWasPressed()){
                toTarget = !toTarget;
            }
            double servoPower = 0;
            if(!toTarget){
                if(gamepad1.y){
                    CrServo.setPower(1);
                }else if(gamepad1.b){
                    CrServo.setPower(-1);
                }else {
                    CrServo.setPower(0);
                }
            }else{
                servoPower = CRPID.calculate(encoder.getVoltage() * 355 /encoder.getMaxVoltage());
                CrServo.setPower(-servoPower);
            }


            telemetry.addData("Pose: ",encoder.getVoltage() * 355 /encoder.getMaxVoltage());
            telemetry.addData("maxPose: ",maxPose);
            telemetry.addData("Voltage OutPut: ",encoder.getVoltage());
            telemetry.addData("max Voltage: ",encoder.getMaxVoltage());
            telemetry.addData("power: ",servoPower);
            telemetry.addData("power f serv: ",CrServo.getPower());
            telemetry.update();




        }
    }
}
