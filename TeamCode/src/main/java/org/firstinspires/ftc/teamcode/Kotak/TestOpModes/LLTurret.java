package org.firstinspires.ftc.teamcode.Kotak.TestOpModes;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.Utils.asmPIDController;


@Config
@TeleOp(name = "Turret Calibration",group = "Competition")
public class LLTurret extends LinearOpMode {
    private DcMotorEx turretMotor = null;
    private asmGamepadEx driver1;

    private Limelight3A  limelight;

    private asmPIDController turretPIDController = null;

    public static double kP = 0.001;
    public static double kD = 0;

    public static double tolerance = 1;

    private Follower follower;
    @Override
    public void runOpMode() {
        turretPIDController = new asmPIDController(kP,0,kD);
        turretPIDController.setTolerance(tolerance);
        turretPIDController.setTarget(0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor = hardwareMap.get(DcMotorEx.class,"turret");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        telemetry.setMsTransmissionInterval(1);

        limelight.start();

        waitForStart();

        while (opModeIsActive()){


            if(gamepad1.right_bumper){
                turretMotor.setPower(1);
            }else if(gamepad1.left_bumper){
                turretMotor.setPower(-1);
            }else{
                turretMotor.setPower(0);
            }

            LLResult result = limelight.getLatestResult();

            if(result != null){
                if(result.isValid()){
                    telemetry.addData("Tx: ",result.getTx());
                    telemetry.addData("id: ",result.getPipelineIndex());
                    turretMotor.setPower(turretPIDController.calculate(result.getTx()));
                }
            }

            telemetry.addData("turret Pose",turretMotor.getCurrentPosition());
            telemetry.update();

        }
    }

}
