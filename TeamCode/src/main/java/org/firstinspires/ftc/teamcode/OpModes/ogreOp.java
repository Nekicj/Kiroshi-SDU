package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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


@Config
@TeleOp(name = "Solo TelePopus",group = "Competition")
public class ogreOp extends LinearOpMode {
    private Niggantroller niggantroller;
    private BaseController baseController;
    private asmGamepadEx driver1;
    private asmRobotState robotState = new asmRobotState();

    private double targetVelocityToCheck = asmConfig.motorVelocityClose ;
    private double offset = asmConfig.motorOffsetClose;
    private boolean toArtifact = false;
    private boolean isCloseScore = true;
    private boolean isShooting = false;
    private boolean isTurretFieldCentric = true;
    private boolean isRobotCentric = false;
    private boolean isBlue = false;
    private boolean isTurretNull = false;


    private PathChain pathToScore = null;
    private Pose poseScore = new Pose(0,0,0);

    private double targetTurretAngleCloseFieldCentric = asmConfig.targetTurretCloseFieldCentric;
    private double targetTurretAngleLongFieldCentric = asmConfig.targetTurretLongFieldCentric;

    private double targetTurretAngleCloseRobotCentric = 0;
    private double targetTurretAngleLongRobotCentric = asmConfig.targetTurretLongRobotCentric;

    private Pose poseAfterAuto = new Pose(0,0,0);
    private boolean isPoseUpdated = false;
    public static double yawScalar = 1.000477;

    private Follower follower;
    @Override
    public void runOpMode() {
        isBlue = asmConfig.isBlue;
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleopDrive(true);
//        poseAfterAuto = new Pose(robotState.getPoseAfterAuto().getX(),robotState.getPoseAfterAuto().getY(),0);
        follower.setStartingPose(poseAfterAuto);
        follower.setPose(poseAfterAuto);

        driver1 = new asmGamepadEx(gamepad1);

        niggantroller = new Niggantroller(hardwareMap,telemetry);
        niggantroller.setTurretGamepad(gamepad1);
        niggantroller.setTurretMode(TurretController.TurretMode.FIELD_ANGLE);

        if(isBlue){

        }

        niggantroller.setFieldAngleTarget(targetTurretAngleCloseFieldCentric);
        niggantroller.setRobotRelativeAngle(targetTurretAngleLongFieldCentric);

        baseController = new BaseController();
        baseController.initialize(hardwareMap, true);
        baseController.resetHeading(yawScalar);



        telemetry.addData("Status, ","Initialized");
        telemetry.addData("Pose: ",follower.getPose().toString());


//        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
//        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        waitForStart();

        while (opModeIsActive()){
//            hubs.forEach(LynxModule::clearBulkCache);
            niggantroller.setFieldAngleTarget(targetTurretAngleCloseFieldCentric);
            niggantroller.setRobotRelativeAngle(targetTurretAngleLongFieldCentric);

            if(isBlue){
                targetTurretAngleCloseFieldCentric = asmConfig.targetTurretCloseFieldCentricBlue;
                targetTurretAngleLongFieldCentric = asmConfig.targetTurretLongFieldCentricBlue;

                targetTurretAngleCloseRobotCentric = 0;
                targetTurretAngleLongRobotCentric = asmConfig.targetTurretLongRobotCentricBlue;
            }else{
                targetTurretAngleCloseFieldCentric = asmConfig.targetTurretCloseFieldCentric;
                targetTurretAngleLongFieldCentric = asmConfig.targetTurretLongFieldCentric;

                targetTurretAngleCloseRobotCentric = 0;
                targetTurretAngleLongRobotCentric = asmConfig.targetTurretLongRobotCentric;
            }



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

            if(!driver1.isLeftTriggerDown()){
                follower.setTeleOpDrive(
                        forward ,
                        strafe ,
                        rotate *0.6,
                        isRobotCentric
                );
            }else{
                follower.setTeleOpDrive(
                        0 ,
                        0 ,
                        0,
                        isRobotCentric
                );
            }

            if(!isPoseUpdated){
                follower.setPose(poseAfterAuto);
                isPoseUpdated = true;
            }


            if(driver1.isXPressed()){
                isShooting = !isShooting;
            }

            if(driver1.isAPressed()){
                isTurretFieldCentric = !isTurretFieldCentric;
            }



            if(driver1.isYPressed()){
                isRobotCentric = !isRobotCentric;
            }

            if(driver1.isBPressed()){

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
                niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoClose);
                if(isTurretFieldCentric){
                    niggantroller.setTurretMode(TurretController.TurretMode.FIELD_ANGLE);
                    niggantroller.setFieldAngleTarget(targetTurretAngleCloseFieldCentric);
                }else{
                    niggantroller.setTurretMode(TurretController.TurretMode.ROBOT_RELATIVE);
                    niggantroller.setFieldAngleTarget(targetTurretAngleCloseRobotCentric);
                }


                targetVelocityToCheck = asmConfig.motorVelocityClose;
                offset = asmConfig.motorOffsetClose;
                niggantroller.setShooterVelocity(targetVelocityToCheck);
            }else{

                targetVelocityToCheck = asmConfig.motorVelocityLong;
                offset = asmConfig.motorOffsetLong;
                niggantroller.setShooterVelocity(targetVelocityToCheck);
                niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoHigh);

                if(isTurretFieldCentric){
                    niggantroller.setTurretMode(TurretController.TurretMode.FIELD_ANGLE);
                    niggantroller.setFieldAngleTarget(targetTurretAngleLongFieldCentric);
                }else{
                    niggantroller.setTurretMode(TurretController.TurretMode.ROBOT_RELATIVE);
                    niggantroller.setFieldAngleTarget(targetTurretAngleLongRobotCentric);
                }
            }

            if(driver1.isRightStickButtonPressed()){
                isCloseScore = true;
            }else if(driver1.isLeftStickButtonPressed()){
                isCloseScore = false;
            }

            if(driver1.isBackPressed()){
//                baseController.resetHeading();
                Pose followerPose = follower.getPose();
                baseController.resetHeading(yawScalar);
                follower.setPose(new Pose(followerPose.getX(),followerPose.getY(),0));
            }

            if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset)){
                gamepad1.rumble(0.1,0.1,50);
            }



            niggantroller.update(gamepad2.back);
            niggantroller.updateTurret(follower.getPose());
            robotState.updatePose(follower.getPose());
//            niggantroller.showTurretTelemetry(telemetry);
            telemetry.addData("X",follower.getPose().getX());
            telemetry.addData("Y",follower.getPose().getY());
            telemetry.addData("heading",follower.getPose().getHeading());
            niggantroller.showShooterTelemetry(telemetry);

            telemetry.update();

        }
    }

}
