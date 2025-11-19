package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Niggantroller;
import org.firstinspires.ftc.teamcode.Controllers.BaseController;
import org.firstinspires.ftc.teamcode.Controllers.ShooterControllerPIDVSA;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;
import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.Utils.asmRobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;


@Config
@TeleOp(name = "Solo TelePopus",group = "Competition")
public class ogreOp extends LinearOpMode {
    private Niggantroller niggantroller;
    private BaseController baseController;
    private asmGamepadEx driver1;
    private asmRobotState robotState = new asmRobotState();

    private double targetVelocityToCheck = asmConfig.motorVelocityCloseTeleop;
    private double offset = asmConfig.motorOffsetCloseTeleop;
    private boolean toArtifact = false;
    private boolean isCloseScore = true;
    private boolean isShooting = false;

    private PathChain pathToScore = null;
    private Pose poseScore = new Pose(0,0,0);

    public static double targetTurretAngle = 39;

    private Follower follower;
    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleopDrive(true);
        follower.setStartingPose(robotState.getPoseAfterAuto());

        driver1 = new asmGamepadEx(gamepad1);

        niggantroller = new Niggantroller(hardwareMap,telemetry);
        niggantroller.setTurretGamepad(gamepad1);
        niggantroller.setTurretMode(TurretController.TurretMode.FIELD_ANGLE);
        niggantroller.setFieldAngleTarget(targetTurretAngle);
        niggantroller.setRobotRelativeAngle(targetTurretAngle);

        baseController = new BaseController();
        baseController.initialize(hardwareMap,true);


        telemetry.addData("Status, ","Initialized");
        telemetry.addData("Pose: ",follower.getPose().toString());


//        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
//        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        waitForStart();

        while (opModeIsActive()){
//            hubs.forEach(LynxModule::clearBulkCache);
            niggantroller.setFieldAngleTarget(targetTurretAngle);
            niggantroller.setRobotRelativeAngle(targetTurretAngle);
            driver1.update();

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

            follower.setTeleOpDrive(
                    forward ,
                    strafe ,
                    rotate *0.6,
                    false
            );

            if(driver1.isXPressed()){
                isShooting = !isShooting;
            }


            if(isShooting){
                niggantroller.toShoot(true);
//                niggantroller.setTurretAutoAimEnabled(true);
            }else{
                niggantroller.toShoot(false);
//                niggantroller.setTurretAutoAimEnabled(false);
            }


            if(driver1.isRightBumperPressed()){
                niggantroller.intakeEpt(1);
            }
            if(driver1.isLeftBumperPressed()){
                niggantroller.intakeEpt(-1);
            }
            if(driver1.isDpadUpPressed()){
                niggantroller.intakeEpt(0);
            }
            if(driver1.isRightTriggerPressed(0.2)){
                niggantroller.shootBall();
            }

            if(isCloseScore){
                niggantroller.setDirectionPos(ShooterControllerPIDVSA.ServosPos.DIRECTION_DOWN.getPos());
                targetVelocityToCheck = asmConfig.motorVelocityClose;
                offset = asmConfig.motorOffsetClose;
                niggantroller.setShooterVelocity(targetVelocityToCheck);
            }else{
                targetVelocityToCheck = asmConfig.motorVelocityLong;
                offset = asmConfig.motorOffsetLong;
                niggantroller.setShooterVelocity(targetVelocityToCheck);
                niggantroller.setDirectionPos(ShooterControllerPIDVSA.ServosPos.DIRECTION_DOWN.getPos());
            }

            if(driver1.isRightStickButtonPressed()){
                isCloseScore = true;
            }else if(driver1.isLeftStickButtonPressed()){
                isCloseScore = false;
            }

            if(driver1.isBackPressed()){
                baseController.resetHeading();
                follower.setStartingPose(new Pose(0,0,0));
            }

            if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset)){
                gamepad1.rumble(0.1,0.1,50);
            }



            niggantroller.update(gamepad2.back);
            niggantroller.updateTurret(follower.getPose());
//            niggantroller.showTurretTelemetry(telemetry);
            telemetry.addData("X",follower.getPose().getX());
            telemetry.addData("Y",follower.getPose().getY());
            telemetry.addData("heading",follower.getPose().getHeading());
            niggantroller.showShooterTelemetry(telemetry);

            telemetry.update();

        }
    }

}
