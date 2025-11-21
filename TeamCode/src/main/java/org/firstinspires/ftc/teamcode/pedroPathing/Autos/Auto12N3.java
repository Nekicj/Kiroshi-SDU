package org.firstinspires.ftc.teamcode.pedroPathing.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.BaseController;
import org.firstinspires.ftc.teamcode.Controllers.Niggantroller;
import org.firstinspires.ftc.teamcode.Controllers.ShooterControllerPIDVSA;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;
import org.firstinspires.ftc.teamcode.Utils.asmRobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "12+3",group = "Competition")
public class Auto12N3 extends OpMode {
    private Follower follower;
    private Timer pathTimer,acitionTimer,opModeTimer;
    private Niggantroller niggantroller;
    private asmRobotState robotState = new asmRobotState();;

    private BaseController baseController = null;

    private int pathState = 0;
    private int ballCount = 0;

    private boolean isBlue = false;

    private double targetTurretAngle = 128;
    private double targetTurretAngleLong = -25;
    public static double setShooterVelocityAutoLong = 1520;

    public static double ShooterVelocityClose = 1150; //1145


    private  Pose nullPose = new Pose(0,0,0);
    private  Pose startPose = null;

    //    private final Pose scorePose = new Pose(28.854,-8.513,0);
    private  Pose scorePose = null;

    private  Pose take1PosePath = null;
    private Pose gate= null;

    private  Pose take1PoseStart = null;
    private  Pose take1PoseFinal = null;

    private  Pose take2PosePath = null;

    private  Pose take2PoseStart = null;
    private  Pose take2PoseFinal = null;

    private  Pose take3PosePath = null;

    private  Pose take3PoseStart = null;
    private  Pose take3PoseFinal = null;

    private  Pose parking = null;

    private  Pose pathToTake5 = null;
    private  Pose take5Final = null;
    private  Pose shootPose5 = null;





    public PathChain startToScore1,score1ToTake1,take2ToScore2,scoreToTake2,take2ToScore,score2ToTake3,take3ToScore,take1ToScore,scoreToParking,scoreToShoot5;

    public ElapsedTime niggtimer;
    public ElapsedTime balltimer;
    public void buildPaths(){
        startToScore1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading(),0.7)
                .setBrakingStrength(0.5)
                .setBrakingStart(0.7)
                .build();

//        score1ToTake1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose,take1PosePath))
//                .setTangentHeadingInterpolation()
//                .setTValueConstraint(0.8)
//                .addPath(new BezierPoint(take1PoseStart))
//                .addPath(new BezierLine(take1PoseStart,take1PoseFinal))
//                .setTangentHeadingInterpolation()
//                .setTValueConstraint(0.8)
//                .build();
        score1ToTake1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,take1PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),take1PoseStart.getHeading())
                .setTValueConstraint(0.8)
                .addPath(new BezierLine(take1PoseStart,take1PoseFinal))
                .setTangentHeadingInterpolation()
                .setTValueConstraint(0.8)
                .build();


        take1ToScore = follower.pathBuilder()
                .addPath(new BezierLine(take1PoseFinal,scorePose))
                .setLinearHeadingInterpolation(take1PoseFinal.getHeading(),scorePose.getHeading(),0.7)
                .setBrakingStart(0.7)
                .setBrakingStrength(0.5)
                .build();

//        scoreToTake2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose,take2PosePath))
//                .setTangentHeadingInterpolation()
//                .setTValueConstraint(0.8)
//                .addPath(new BezierPoint(take2PoseStart))
//                .addPath(new BezierLine(take2PoseStart,take2PoseFinal))
//                .setTangentHeadingInterpolation()
//                .setTValueConstraint(0.8)
//                .build();

        scoreToTake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,take2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),take2PoseStart.getHeading())
                .setTValueConstraint(0.8)
                .addPath(new BezierLine(take2PoseStart,take2PoseFinal))
                .setTangentHeadingInterpolation()
                .setTValueConstraint(0.8)
                .build();



        if(isBlue){
            take2ToScore = follower.pathBuilder()
                    .addPath(new BezierLine(take2PoseFinal,gate))
                    .setLinearHeadingInterpolation(take2PoseFinal.getHeading(),gate.getHeading())

                    .addPath(new BezierLine(gate,scorePose))
                    .setLinearHeadingInterpolation(gate.getHeading(),scorePose.getHeading(),0.7)
                    .setBrakingStart(0.7)
                    .setBrakingStrength(0.5)
                    .setTValueConstraint(0.8)
                    .build();
        }else{
            take2ToScore = follower.pathBuilder()
                    .addPath(new BezierLine(take2PoseFinal,scorePose))
                    .setLinearHeadingInterpolation(take2PoseFinal.getHeading(),scorePose.getHeading(),0.7)
                    .setBrakingStart(0.7)
                    .setBrakingStrength(0.5)
                    .build();
        }

//        score2ToTake3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose,take3PosePath))
//                .setTangentHeadingInterpolation()
//                .setTValueConstraint(0.8)
//                .addPath(new BezierPoint(take3PoseStart))
//                .addPath(new BezierLine(take3PoseStart,take3PoseFinal))
//                .setTangentHeadingInterpolation()
//                .setTValueConstraint(0.8)
//                .build();

        score2ToTake3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,take3PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),take3PoseStart.getHeading())
                .setTValueConstraint(0.8)
                .addPath(new BezierLine(take3PoseStart,take3PoseFinal))
                .setTangentHeadingInterpolation()
                .setTValueConstraint(0.8)
                .build();


        take3ToScore = follower.pathBuilder()
                .addPath(new BezierLine(take3PoseFinal,scorePose))
                .setLinearHeadingInterpolation(take3PoseFinal.getHeading(),scorePose.getHeading(),0.7)
                .setBrakingStart(0.7)
                .setBrakingStrength(0.5)
                .build();

        scoreToShoot5 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pathToTake5))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pathToTake5.getHeading(),0.7)
                .setTValueConstraint(0.8)

                .addPath(new BezierLine(pathToTake5,take5Final))
                .setConstantHeadingInterpolation(take5Final.getHeading())
                .setTValueConstraint(0.8)

                .addPath(new BezierLine(take5Final,shootPose5))
                .setLinearHeadingInterpolation(take5Final.getHeading(),shootPose5.getHeading(),0.7)
                .setBrakingStart(0.7)
                .setBrakingStrength(0.7)

                .build();

        scoreToParking = follower.pathBuilder()
                .addPath(new BezierLine(shootPose5,take2PoseStart))
                .setLinearHeadingInterpolation(shootPose5.getHeading(),take2PoseStart.getHeading(),0.7)
                .build();





    }

    @Override
    public void init(){
        isBlue = asmConfig.isBlue;

        baseController = new BaseController();
        baseController.initialize(hardwareMap,true);
        baseController.resetHeading(1);

        niggtimer = new ElapsedTime();
        balltimer = new ElapsedTime();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        niggantroller = new Niggantroller(hardwareMap,telemetry);



        if(isBlue){
            startPose = new Pose(2.224,-23.19,-2.30);

            scorePose = new Pose(26.838,5.141,-0);

            take1PoseStart = new Pose(38.61,-3.483,-1.57);
            take1PoseFinal = new Pose(38.61,-25.8,-1.57);

            take2PoseStart = new Pose(65.019,-1.9428,-1.57);
            take2PoseFinal = new Pose(65.019,-30.8,-1.57);

            gate = new Pose(56,-27.377,0);

            take3PoseStart = new Pose(84.9,-1.78,-1.57);
            take3PoseFinal = new Pose(84.9,-30,-1.57);

            pathToTake5 = new Pose(81.25,-33.631,-0.64);
            take5Final = new Pose(110,-35.4,-0.64);

            shootPose5 = new Pose(111.17,15.15,-3.15);

            targetTurretAngle *= -1;
            targetTurretAngle -= 6;
            targetTurretAngleLong *=-1;
            targetTurretAngleLong -= 3;

        }else{
            nullPose = new Pose(0,0,0);
            startPose = new Pose(5.56,19.21,2.36);

//    private final Pose scorePose = new Pose(28.854,-8.513,0);
            scorePose = new Pose(28.854,-8.513,0);

            take1PosePath = new Pose(43.7,1.59,0);

            take1PoseStart = new Pose(37.24,-8.513,1.57);
            take1PoseFinal = new Pose(37.24,24.8,1.57);

            take2PosePath = new Pose(67,1.59,0);

            take2PoseStart = new Pose(68.23,-8.513,1.57);
            take2PoseFinal = new Pose(68.23,28.8,1.57);

            take3PosePath = new Pose(90,1.59,0);

            take3PoseStart = new Pose(92,-8.513,1.57);
            take3PoseFinal = new Pose(92,27.8,1.57);

            parking = new Pose(30,7.5,1.57);

            pathToTake5 = new Pose(80.64,29.07,0.84);
            take5Final = new Pose(114.37,27.429,0.84);
            shootPose5 = new Pose(115.802,-24.434,-3.16);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);



        buildPaths();
    }

    public void pathUpdate(){
        switch (pathState){
            case 0:
                if(!follower.isBusy()){
                    follower.followPath(startToScore1);
                    pathState = 100;
                    niggantroller.intakeEpt(1);
                    niggantroller.setShooterVelocity(ShooterVelocityClose);

                    niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoClose);
                    niggantroller.toShoot(true);
                }
                break;
            case 100:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    balltimer.reset();
                    pathState = 1;

                }
                break;
            case 1:
                if(!follower.isBusy() && niggtimer.milliseconds() > 900){
                    niggantroller.intakeEpt(0);
                    pathState = 2 ;
                    niggtimer.reset();
                }
                break;
            case 2:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1300){
                    follower.followPath(score1ToTake1);
                    pathState = 3;
                    niggantroller.toShoot(false);
                    niggantroller.intakeEpt(1);

                }
                break;
            case 3:
                if(!follower.isBusy()){
                    pathState = 4;
                    niggtimer.reset();
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(take1ToScore);
                    pathState = 101;

                    niggantroller.setShooterVelocity(ShooterVelocityClose);
                    niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoClose);
                    niggantroller.toShoot(true);
                }
                break;
            case 101:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    balltimer.reset();
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy() && niggtimer.milliseconds() > 900){
                    niggantroller.intakeEpt(0);
                    pathState = 6;
                    niggtimer.reset();
                }
                break;
            case 6:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1300){
                    follower.followPath(scoreToTake2);
                    niggantroller.toShoot(false);
                    niggantroller.intakeEpt(1);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.followPath(take2ToScore);

                    niggantroller.setShooterVelocity(ShooterVelocityClose);
                    niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoClose);
                    niggantroller.toShoot(true);
                    pathState = 102;
                }
                break;
            case 102:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    balltimer.reset();
                    pathState = 8;
                }
                break;
            case 8:
                if(!follower.isBusy() && niggtimer.milliseconds() > 900){
                    niggantroller.intakeEpt(0);
                    pathState = 9;
                    niggtimer.reset();
                }
                break;
            case 9:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1300){
                    follower.followPath(score2ToTake3);
                    niggantroller.toShoot(false);
                    niggantroller.intakeEpt(1);
                    pathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    follower.followPath(take3ToScore);

                    niggantroller.setShooterVelocity(ShooterVelocityClose);
                    niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoClose);
                    niggantroller.toShoot(true);
                    pathState = 103;
                }
                break;
            case 103:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    balltimer.reset();
                    pathState = 11;
                }
                break;
            case 11:
                if(!follower.isBusy() && niggtimer.milliseconds() > 900){
                    niggantroller.intakeEpt(0);
                    pathState = 12;
                    niggtimer.reset();

                }
                break;
            case 12:
                if(!follower.isBusy()&&niggtimer.milliseconds() > 1300){
                    pathState = 13;
                    follower.followPath(scoreToShoot5);

                    niggantroller.setShooterVelocity(setShooterVelocityAutoLong);

                    niggantroller.toShoot(true);
                    niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoHigh);
                    niggantroller.intakeEpt(1);
                    targetTurretAngle = targetTurretAngleLong;
//                    niggantr 6i9-+oller.setShooterVelocity(setShooterVelocityAutoLong);
//                    shooterVelocityAuto = 1900;
                }
                break;
            case 13:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    balltimer.reset();
                    pathState =14;
                }
                break;
            case 14:
                if(!follower.isBusy() && niggtimer.milliseconds() > 700){
                    if(niggantroller.checkShooterVelocity(setShooterVelocityAutoLong,20) && niggtimer.milliseconds() > 250){
                        if(ballCount <3){
                            niggantroller.shootBall();
                            niggtimer.reset();
                            ballCount+=1;
                        }else{
                            niggantroller.toShoot(false);
//                            follower.activateAllPIDFs();
                            ballCount = 0;
                            pathState = 15;
                            niggtimer.reset();
                        }
                    }

                }
                break;
            case 15:
                if(!follower.isBusy()){
                    follower.followPath(scoreToParking);
                    targetTurretAngle = 20;
                    pathState =16;
                }
                break;



        }
    }
    @Override
    public void loop(){
        follower.update();
        pathUpdate();


        niggantroller.update(false);
        niggantroller.updateTurret(follower.getPose());
        niggantroller.setTurretMode(TurretController.TurretMode.ROBOT_RELATIVE);



//        niggantroller.setFieldAngleTarget(targetTurretAngle);
        niggantroller.setRobotRelativeAngle(targetTurretAngle);
//        niggantroller.showTurretTelemetry(telemetry);
        niggantroller.setTurretAutoAimEnabled(true);
        niggantroller.showShooterTelemetry(telemetry);

        robotState.updatePose(follower.getPose());

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }
}
