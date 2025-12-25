package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Niggantroller;
import org.firstinspires.ftc.teamcode.Controllers.BaseController;
import org.firstinspires.ftc.teamcode.Controllers.ShooterControllerPIDVSA;
import org.firstinspires.ftc.teamcode.Controllers.TurretControllers.TurretControllerMotor;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;
import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.Utils.asmRobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Grind it, chop it, mix it, flava-it
 * Bitch, I get it on my mind
 * Smoke it, roll it, taste it, bang it
 * Bitch, I can't get even high
 * You can't get way that I'm flexin'
 * Bitch, I'm on my demon time
 * You know this my flow is extendin'
 * I grab the shit of my crime
 *
 * Сломанный ублюдок — тот, кем мне никогда не стать
 * Карманы жрут купюры, стоило мне лишь начать
 * Словно Боинг, сука
 * manera создан, чтоб летать
 * Мне не тянули руку
 * Я сам научился сиять
 *
 * Grind it, chop it, mix it, flava-it
 * Bitch, I get it on my mind
 * Smoke it, roll it, taste it, bang it
 * Bitch, I can't get even high
 * You can't get way that I'm flexin'
 * Bitch, I'm on my demon time
 * You know this my flow is extendin'
 * I'm grab the shit of my crime
 * Grind it, chop it, mix it, flava-it
 * Bitch, I get it on my mind
 * Smoke it, roll it, taste it, bang it
 * Bitch, I can't get even high
 * You can't get way that I'm flexin'
 * Bitch, I'm on my demon time
 * You know this my flow is еxtendin'
 * I'm grab the shit of my crime
 *
 * Кто бы мог подумать, а
 * Lil' shawty точно влюблена
 * Тупит, но вроде не дура, а
 * Запрыгнет со мной в бумер, а
 * Я видел это тыщу раз
 * Две тыщи слёз из тыщи глаз
 * manеra bless, топлю на газ
 * Ты знаешь, что скажу сейчас
 *
 * Grind it, chop it, mix it, flava-it
 * Bitch, I get it on my mind
 * Smoke it, roll it, taste it, bang it
 * Bitch, I can't get even high
 * You can't get way that I'm flexin'
 * Bitch, I'm on my demon time
 * You know this my flow is extendin'
 * I'm grab the shit of my crime
 */

@Config
@TeleOp(name = "Solo TelePopus",group = "Competition")
public class ogreOp extends LinearOpMode {
    private Niggantroller niggantroller;
    private BaseController baseController;
    private asmGamepadEx driver1;
    private asmRobotState robotState = new asmRobotState();
    private TurretControllerMotor turretController;

    private double targetVelocityToCheck = asmConfig.motorVelocityClose ;
    private double offset = asmConfig.motorOffsetClose;
    private boolean isCloseScore = true;
    private boolean isShooting = false;
    private boolean isTurretFieldCentric = false;
    private boolean isRobotCentric = false;
    private boolean isBlue = false;
    private boolean isRamp = false;

    private Pose poseAfterAuto = new Pose(0,0,0);
//    public static double yawScalar = 1.000477;

    private Follower follower;
    @Override
    public void runOpMode() {
        isBlue = asmConfig.isBlue;
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleopDrive(true);
        follower.setStartingPose(poseAfterAuto);
        follower.setPose(poseAfterAuto);

        driver1 = new asmGamepadEx(gamepad1);

        niggantroller = new Niggantroller(hardwareMap,telemetry);

        baseController = new BaseController();
        baseController.initialize(hardwareMap, true);
        baseController.resetHeading(1);

        turretController = new TurretControllerMotor();
        turretController.initialize(hardwareMap,"turret");

        turretController.setTurretMode(TurretControllerMotor.TurretMode.FIELD_TARGET);
        turretController.setFieldAngleTarget(asmConfig.targetTurretCloseFieldCentric);



        telemetry.addData("Status, ","Initialized");
        telemetry.addData("Pose: ",follower.getPose().toString());

        waitForStart();

        while (opModeIsActive()){
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


            if(driver1.isXPressed()){
                isShooting = !isShooting;
            }

            if(driver1.isAPressed()){
                isTurretFieldCentric = !isTurretFieldCentric;
            }

            if(isTurretFieldCentric){
                turretController.setTurretMode(TurretControllerMotor.TurretMode.FIELD_ANGLE);
            }else{
                turretController.setTurretMode(TurretControllerMotor.TurretMode.FIELD_TARGET);

            }



            if(driver1.isYPressed()){
                isRobotCentric = !isRobotCentric;
            }


            if(isShooting){
                niggantroller.toShootShooter(true);
//                niggantroller.setTurretAutoAimEnabled(true);
            }else{
                niggantroller.toShootShooter(false);
//                niggantroller.setTurretAutoAimEnabled(false);
            }


            if(driver1.isRightBumperPressed()){
                niggantroller.intakeEpt(-1);
                niggantroller.shootBall(false);
            }
            if(driver1.isLeftBumperPressed()){
                niggantroller.intakeEpt(1);
            }
            if(driver1.isRightTriggerPressed(0.15)){
                niggantroller.shootBall(true);

            }

            if(driver1.isBPressed()){
                isRamp = !isRamp;
                niggantroller.setRamp(isRamp);
            }





            if(isCloseScore){
                niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoClose);
                turretController.setFieldAngleTarget(asmConfig.targetTurretCloseFieldCentric);


                targetVelocityToCheck = asmConfig.motorVelocityClose;
                offset = asmConfig.motorOffsetClose;
                niggantroller.setShooterVelocity(targetVelocityToCheck);
            }else{
                turretController.setFieldAngleTarget(asmConfig.targetTurretLongFieldCentric);

                targetVelocityToCheck = asmConfig.motorVelocityLong;
                offset = asmConfig.motorOffsetLong;
                niggantroller.setShooterVelocity(targetVelocityToCheck);
                niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoHigh);
            }

            if(driver1.isRightStickButtonPressed()){
                isCloseScore = true;
            }else if(driver1.isLeftStickButtonPressed()){
                isCloseScore = false;
            }

            if(driver1.isBackPressed()){
                baseController.resetHeading(1);
//                Pose followerPose = follower.getPose();
//                baseController.resetHeading(yawScalar);
//                follower.setPose(new Pose(followerPose.getX(),followerPose.getY(),0));
            }

            if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset)){
                gamepad1.rumble(0.1,0.1,50);
            }



            niggantroller.update(gamepad2.back);
            turretController.update(follower.getPose());
            turretController.showTelemetry(telemetry);
//            robotState.updatePose(follower.getPose());
//            niggantroller.showTurretTelemetry(telemetry);
            telemetry.addData("X",follower.getPose().getX());
            telemetry.addData("Y",follower.getPose().getY());
            telemetry.addData("heading",follower.getPose().getHeading());
//            niggantroller.showShooterTelemetry(telemetry);

            telemetry.update();

        }
    }

}
