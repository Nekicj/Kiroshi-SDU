package org.firstinspires.ftc.teamcode.pedroPathing.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Niggantroller;
import org.firstinspires.ftc.teamcode.Controllers.ShooterControllerPIDVSA;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Config
@Disabled
@Autonomous(name = "12+0 gate n curve",group = "Competition Auto")
public class ObsidianAutoNCurveNGate extends OpMode {
    private static final Logger log = LoggerFactory.getLogger(ObsidianAutoNCurveNGate.class);
    private Niggantroller niggantroller;
    private Follower follower;
    private Timer pathTimer,acitionTimer,opModeTimer;
    private ElapsedTime niggTimer;
    private asmConfig asmconfig = null;

    public static boolean isLongScore = false;

    private int pathState = 0;

    private int ballsCount = 0;

    public static double pattern = 0;


    // 0 - GPP
    // 1 - PGP
    // 2 - PPG


    private Pose scorePose = null;

    private Pose startPose = null;

    private Pose closeScorePose = null;
    private Pose longScore = null;

    private Pose take1PosStart = null;
    private Pose take1PosEnd = null;

    private Pose take2PosStart = null;
    private Pose take2PosEnd = null;

    private Pose take3PosStart = null;
    private Pose take3PosEnd = null;

    private Pose parkingPose = null;

    private Pose takingCurvePose = null;

    private Pose take2ToGate = null;

    private Pose take2Gate = null;

    // take1 PPG
    // take2 PGP
    // take3 GPP

    // -1820 -1600 long score

    // -1420 -1200-1220 close score

    private double targetVelocityToCheck = -1820;
    private double offset = -270;

    public static boolean isRed = false;

    public PathChain take1Path;
    public PathChain take2Path;
    public PathChain take3Path;

    public PathChain startToScore;

    public PathChain scoreToTake1;
    public PathChain scoreToTake3;
    public PathChain scoreToTake2;

    public PathChain take1toScore;
    public PathChain take2ToScore;
    public PathChain take3ToScore;

    public PathChain take2ToGatePath;

    public PathChain scoreToParking;

    public void buildPaths(){


        startToScore = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading(),1)

                .build();

        scoreToTake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,takingCurvePose,take1PosStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),take1PosStart.getHeading(),1)
                .build();


//            take1Path = new Path(new BezierLine(take1PosStart,take1PosEnd));
//            take1Path.setConstantHeadingInterpolation(take1PosStart.getHeading());
//        take1Path.setLinearHeadingInterpolation(scorePose.getHeading(),take1PosEnd.getHeading());

        take1Path = follower.pathBuilder()
                .addPath(new BezierLine(take1PosStart,take1PosEnd))
                .setConstantHeadingInterpolation(take1PosStart.getHeading())
                .build();

        take1toScore = follower.pathBuilder()
                .addPath(new BezierCurve(take1PosEnd,scorePose))
                .setLinearHeadingInterpolation(take1PosEnd.getHeading(),scorePose.getHeading(),1)
                .build();

        scoreToTake2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,takingCurvePose,take2PosStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),take2PosStart.getHeading(),1)
                .build();

//            take2Path = new Path(new BezierLine(take2PosStart,take2PosEnd));
//            take2Path.setConstantHeadingInterpolation(take2PosStart.getHeading());
//        take2Path.setLinearHeadingInterpolation(scorePose.getHeading(),take2PosEnd.getHeading());

//        take2Path = follower.pathBuilder()
//                .addPath(new BezierLine(take2PosStart,take2PosEnd))
//                .setLinearHeadingInterpolation(take2PosStart.getHeading(),take2PosEnd.getHeading())
//
//                .addPath(new BezierLine(take2PosEnd,take2ToGate))
//                .setLinearHeadingInterpolation(take2PosEnd.getHeading(),take2ToGate.getHeading())
//
//                .addPath(new BezierLine(take2ToGate,take2Gate))
//                .setLinearHeadingInterpolation(take2ToGate.getHeading(),take2Gate.getHeading())
//
//                .build();

        take2ToGatePath = follower.pathBuilder()
                .addPath(new BezierCurve(take2PosEnd,take2ToGate,take2Gate))
                .setLinearHeadingInterpolation(take2PosEnd.getHeading(),take2Gate.getHeading())
                .build();

        take2Path = follower.pathBuilder()
                .addPath(new BezierLine(take2PosStart,take2PosEnd))
                .setConstantHeadingInterpolation(take2PosStart.getHeading())
                .build();

        take2ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(take2PosEnd,scorePose))
                .setLinearHeadingInterpolation(take2PosEnd.getHeading(),scorePose.getHeading(),1)
                .build();

        scoreToTake3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,takingCurvePose,take3PosStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),take3PosStart.getHeading())
                .build();

//            take3Path = new Path(new BezierLine(take3PosStart,take3PosEnd));
//            take3Path.setConstantHeadingInterpolation(take3PosEnd.getHeading());

        take3Path = follower.pathBuilder()
                .addPath(new BezierLine(take3PosStart,take3PosEnd))
                .setConstantHeadingInterpolation(take3PosEnd.getHeading())
                .build();

        take3ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(take3PosEnd,scorePose))
                .setLinearHeadingInterpolation(take3PosEnd.getHeading(),scorePose.getHeading())
                .build();

        scoreToParking = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,parkingPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),parkingPose.getHeading())
                .build();



    }

    @Override
    public void init(){
        asmconfig = new asmConfig();
        isRed = asmConfig.isBlue;

        if(isRed){
            if(!isLongScore){
                scorePose = asmConfig.closeScorePoseRed;
            }else{
                scorePose = asmConfig.longScoreRed;
            }

            take2ToGate = asmConfig.take2ToGateRed;

            take2Gate = asmConfig.take2GateRed;

            takingCurvePose = asmConfig.takingCurvePoseRed;
            startPose = asmConfig.startPoseRed;

            closeScorePose = asmConfig.closeScorePoseRed;
            longScore = asmConfig.longScoreRed;

            take1PosStart = asmConfig.take1PosStartRed;
            take1PosEnd = asmConfig.take1PosEndRed;

            take2PosStart = asmConfig.take2PosStartRed;
            take2PosEnd = asmConfig.take2PosEndRed;

            take3PosStart = asmConfig.take3PosStartRed;
            take3PosEnd = asmConfig.take3PosEndRed;

            parkingPose = asmConfig.parkingPoseRed;
        }else{

            if(!isLongScore){
                scorePose = asmConfig.closeScorePose;
            }else{
                scorePose = asmConfig.longScore;
            }
            take2ToGate = asmConfig.take2ToGate;

            take2Gate = asmConfig.take2Gate;

            takingCurvePose = asmConfig.takingCurvePose;

            startPose = asmConfig.startPose;

            closeScorePose = asmConfig.closeScorePose;
            longScore = asmConfig.longScore;

            take1PosStart = asmConfig.take1PosStart;
            take1PosEnd = asmConfig.take1PosEnd;

            take2PosStart = asmConfig.take2PosStart;
            take2PosEnd = asmConfig.take2PosEnd;

            take3PosStart = asmConfig.take3PosStart;
            take3PosEnd = asmConfig.take3PosEnd;

            parkingPose = asmConfig.parkingPose;
        }

        niggantroller = new Niggantroller(hardwareMap,telemetry);

        if(!isLongScore){
            scorePose = closeScorePose;
            targetVelocityToCheck = asmConfig.motorVelocityClose;
            offset = asmConfig.motorOffsetClose;
            niggantroller.setShooterVelocity(targetVelocityToCheck);
            niggantroller.setDirectionPos(ShooterControllerPIDVSA.ServosPos.DIRECTION_UP.getPos());
        }else{
            scorePose = longScore;
            targetVelocityToCheck = asmConfig.motorVelocityLong;
            niggantroller.setShooterVelocity(targetVelocityToCheck);
            niggantroller.setDirectionPos(ShooterControllerPIDVSA.ServosPos.DIRECTION_DOWN.getPos());
            offset = asmConfig.motorOffsetLong;
        }

        pattern = asmConfig.pattern;

        pathTimer = new Timer();
        pathTimer.resetTimer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        niggTimer = new ElapsedTime();
        niggTimer.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);



        telemetry.addData("isRed: ",isRed);
        niggantroller.showTelemetry(telemetry);
        telemetry.update();

        buildPaths();
    }



    public void pathUpdate(){
        switch (pathState){
            case 0: // GO TO SCORE THE PRELOAD
                if(!follower.isBusy()){
                    follower.followPath(startToScore);
                    niggantroller.intakeEpt(1);
                    niggantroller.toShoot(true);
                    pathState = 1;
                }
                break;
            case 1: // SCORING
                if(!follower.isBusy()){
//                    follower.breakFollowing();
//                    follower.deactivateAllPIDFs();
                    pathState = 2;
                }
                break;
            case 2: // WAIT FOR SHOOTER AND GO TO TAKE 1
                if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > 650){
                    if(ballsCount <3){
                        niggantroller.shootBall();
                        niggTimer.reset();
                        ballsCount+=1;
                    }else{
                        niggantroller.toShoot(false);
                        follower.activateAllPIDFs();
                        ballsCount = 0;
                        pathState = 3;
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(scoreToTake1);
                    pathState = 4;
                }
                break;
            case 4:
                if(follower.getCurrentTValue() >= 0.9){
                    follower.followPath(take1Path);
                    pathState = 5;
                }
                break;
            case 5:
                if(follower.getCurrentTValue() >= 0.94){
                    follower.followPath(take1toScore);
                    niggantroller.toShoot(true);
                    pathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy()){
//                    follower.breakFollowing();
//                    follower.deactivateAllPIDFs();
                    pathState = 7;
                }
                break;
            case 7:
                if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > 650){
                    if(ballsCount <3){
                        niggantroller.shootBall();
                        niggTimer.reset();
                        ballsCount+=1;
                    }else{
                        niggantroller.toShoot(false);
                        follower.activateAllPIDFs();
                        ballsCount = 0;
                        pathState = 8;
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(scoreToTake2);
                    pathState = 9;
                }
                break;
            case 9:
                if(follower.getCurrentTValue() >= 0.9){
                    follower.followPath(take2Path);
                    pathState = 101;
                }
                break;

            case 101:
                if(follower.getCurrentTValue() >= 0.94){
                    follower.followPath(take2ToGatePath);
                    pathState = 10;
                    niggTimer.reset();
                }
                break;
            case 10:
                if(!follower.isBusy() && niggTimer.milliseconds() > 3000){
                    follower.followPath(take2ToScore);
                    niggantroller.toShoot(true);
                    pathState = 11;
                }
                break;
            case 11:
                if(!follower.isBusy()){
//                    follower.breakFollowing();
//                    follower.deactivateAllPIDFs();
                    pathState = 12;
                }
                break;
            case 12:
                if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > 650){
                    if(ballsCount <3){
                        niggantroller.shootBall();
                        niggTimer.reset();
                        ballsCount+=1;
                    }else{
                        niggantroller.toShoot(false);
                        follower.activateAllPIDFs();
                        ballsCount = 0;
                        pathState = 13;
                    }
                }
                break;
            case 13:
                if(!follower.isBusy()){
                    follower.followPath(scoreToTake3);
                    pathState = 14;
                }
                break;
            case 14:
                if(follower.getCurrentTValue() >= 0.9){
                    follower.followPath(take3Path);
                    pathState = 15;
                }
                break;
            case 15:
                if(follower.getCurrentTValue() >= 0.94){
                    follower.followPath(take3ToScore);
                    niggantroller.toShoot(true);
                    pathState = 16;
                }
                break;
            case 16:
                if(!follower.isBusy()){
//                    follower.breakFollowing();
//                    follower.deactivateAllPIDFs();
                    pathState = 17;
                }
                break;
            case 17:
                if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > 650){
                    if(ballsCount <3){
                        niggantroller.shootBall();
                        niggTimer.reset();
                        ballsCount+=1;
                    }else{
                        niggantroller.toShoot(false);
                        niggantroller.intakeEpt(1);
                        follower.activateAllPIDFs();
                        ballsCount = 0;
                        pathState = 18;
                    }
                }
                break;
            case 18:
                if(!follower.isBusy()){
                    follower.followPath(scoreToParking);
                    pathState = 19;
                }
                break;
            case 19:
                if(!follower.isBusy()){
//                    follower.deactivateAllPIDFs();
//                    follower.breakFollowing();
                    pathState = 20;
                }
                break;

        }
    }
    @Override
    public void loop(){
        follower.update();
        pathUpdate();
        niggantroller.update(false);
        asmconfig.setHeading(follower.getHeading());

        telemetry.addData("path state", pathState);
        telemetry.addData("t valur",follower.getCurrentTValue());
        telemetry.addData("balls count",ballsCount);
        telemetry.addData("pattern",pattern);
        telemetry.addData("isRed: ",isRed);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("target - offset", targetVelocityToCheck - offset);
        niggantroller.showShooterTelemetry(telemetry);
        telemetry.addData("true false:", niggantroller.checkShooterVelocity(targetVelocityToCheck,offset));




        telemetry.update();
    }
}
