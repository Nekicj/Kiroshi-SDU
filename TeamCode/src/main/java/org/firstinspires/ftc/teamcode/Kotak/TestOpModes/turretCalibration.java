package org.firstinspires.ftc.teamcode.Kotak.TestOpModes;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Controllers.TurretControllers.TurretControllerMotorNew;
import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@TeleOp(name = "Turret Calibration",group = "Competition")
public class turretCalibration extends LinearOpMode {
    private DcMotorEx turretMotor = null;
    private asmGamepadEx driver1;

    private Follower follower;
    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotorEx.class,"turret");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.right_bumper){
                turretMotor.setPower(1);
            }else if(gamepad1.left_bumper){
                turretMotor.setPower(-1);
            }else{
                turretMotor.setPower(0);
            }

            telemetry.addData("turret Pose",turretMotor.getCurrentPosition());
            telemetry.update();

        }
    }

}
