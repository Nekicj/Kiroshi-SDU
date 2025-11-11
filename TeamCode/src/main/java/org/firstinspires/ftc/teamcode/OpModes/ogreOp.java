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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;


@Config
@TeleOp(name = "Solo TelePopus",group = "Competition")
public class ogreOp extends LinearOpMode {
    private Niggantroller niggantroller;
    private BaseController baseController;
    private asmGamepadEx driver1;

    private double targetVelocityToCheck = asmConfig.motorVelocityCloseTeleop;
    private double offset = asmConfig.motorOffsetCloseTeleop;
    private boolean toArtifact = false;

    private boolean isCloseScore = true;
    private boolean isShooting = false;

    private PathChain pathToScore = null;
    private Pose poseScore = new Pose(0,0,0);

    private Follower follower;
    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(0,0,asmConfig.headingAfterAuto));



//        pathToScore = follower.pathBuilder()
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(0,0,0))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 1))
//                .build();



        driver1 = new asmGamepadEx(gamepad1);
        niggantroller = new Niggantroller(hardwareMap,telemetry);
        baseController = new BaseController();
        baseController.initialize(hardwareMap,true);


        telemetry.addData("Status, ","Initialized");


        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        waitForStart();

        while (opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
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
                    rotate *0.7,
                    false // Robot Centric
            );


//            if(driver1.isAPressed()){
//                poseScore = follower.getPose();
//            }
//
//            if(driver1.isLeftTriggerPressed(0.2)){
//                pathToScore = follower.pathBuilder()
//                        .addPath(new Path(new BezierLine(follower::getPose, poseScore)))
//                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,poseScore.getHeading(), 1))
//                        .build();
//                follower.followPath(pathToScore);
//            }





//            if(!toArtifact){
//                baseController.update(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x,1,gamepad1.left_trigger > 0,false);
//            }else{
//                baseController.update(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x,1,gamepad1.left_trigger > 0,false);
//            }



            if(driver1.isXPressed()){
                isShooting = !isShooting;
//                niggantroller.setShooterCalibration(false);
            }

            if(gamepad2.rightBumperWasPressed()){
                niggantroller.setTurretPose(TurretController.ServosPos.DIRECTION_UP.getPos());
            }


            if(isShooting){
                niggantroller.toShoot(true);
            }else{
                niggantroller.toShoot(false);
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
                niggantroller.setDirectionPos(ShooterControllerPIDVSA.ServosPos.DIRECTION_UP.getPos());
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

//            actionsController.ravaPiet(gamepad1.right_bumper);
//
//            actionsController.ravaBluet(gamepad1.left_bumper);
//
//            actionsController.toUp(gamepad1.dpad_up);



            niggantroller.update(gamepad2.back);

//            telemetry.addData("Status", "Running");
//            telemetry.addData("gamepad left X",gamepad1.left_stick_x);
//            telemetry.addData("gamepad right X",gamepad1.right_stick_x);
//            telemetry.addData("gamepad left Y",gamepad1.left_stick_y);
            niggantroller.showShooterTelemetry(telemetry);
//            niggantroller.showTurretTelemetry(telemetry);
//            baseController.viewTelemetry(telemetry);
            telemetry.update();

        }
    }

}
