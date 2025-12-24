package org.firstinspires.ftc.teamcode.Kotak.TestOpModes;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.BaseController;
import org.firstinspires.ftc.teamcode.Controllers.TurretControllers.TurretControllerMotorNew;
import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@TeleOp(name = "Turret Test",group = "Competition")
public class turretTest extends LinearOpMode {
    private TurretControllerMotorNew turretController;
    private asmGamepadEx driver1;

    private Follower follower;
    @Override
    public void runOpMode() {
        turretController = new TurretControllerMotorNew();
        turretController.initialize(hardwareMap,"turret");

        turretController.setTurretMode(TurretControllerMotorNew.TurretMode.FIELD_TARGET);
//        turretController.setFieldAngleTarget(30);


        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.setStartingPose(new Pose(0,0,0));
        follower.startTeleopDrive(true);

        waitForStart();

        while (opModeIsActive()){

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            if(forward <= 0.05 && forward >= -0.05){
                forward = 0;
            }
            if(strafe <= 0.05 && strafe >= -0.05){
                strafe = 0;
            }
            if(rotate <= 0.05 && rotate>= -0.05){
                rotate = 0;
            }

            follower.update();

            if(true){
                follower.setTeleOpDrive(
                        forward ,
                        strafe ,
                        rotate *0.6,
                        false
                );
            }else{
                follower.setTeleOpDrive(
                        0 ,
                        0 ,
                        0,
                        false
                );
            }

            turretController.update(follower.getPose());

            turretController.showTelemetry(telemetry);
            telemetry.addData("heading ",follower.getPose().getHeading());
            telemetry.update();

        }
    }

}
