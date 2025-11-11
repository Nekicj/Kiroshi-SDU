package org.firstinspires.ftc.teamcode.pedroPathing.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Niggantroller;
import org.firstinspires.ftc.teamcode.Controllers.ShooterControllerPIDVSA;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "12+2 n 7 pattern",group = "Competition Auto")
public class ObsidianWith2PatternAuto extends OpMode {
    private Niggantroller niggantroller;
    private Follower follower;
    private Timer pathTimer,acitionTimer,opModeTimer;
    private ElapsedTime niggTimer;

    public static boolean isLongScore = false;
    public static boolean isRed = false;

    private int pathState = 0;

    private int ballsCount = 0;

    public static int pattern = 0;


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

    private Pose poseToTaking;

    // take1 PPG
    // take2 PGP
    // take3 GPP

    // -1820 -1600 long score

    // -1420 -1200-1220 close score

    private double targetVelocityToCheck = -1820;
    private double offset = -270;

    public static double timeBetweenShoot = 400;

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

    public PathChain scoreToParking;





    public void buildPaths(){
        startToScore = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading(),1)

                .build();

        scoreToTake1 = follower.pathBuilder()


                .addPath(new BezierCurve(scorePose,poseToTaking))
                .setLinearHeadingInterpolation(scorePose.getHeading(),poseToTaking.getHeading(),1)

                .addPath(new BezierCurve(poseToTaking,take1PosStart))
                .setLinearHeadingInterpolation(poseToTaking.getHeading(),take1PosStart.getHeading(),1)

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
                .addPath(new BezierCurve(scorePose,poseToTaking))
                .setLinearHeadingInterpolation(scorePose.getHeading(),poseToTaking.getHeading(),1)

                .addPath(new BezierCurve(poseToTaking,take2PosStart))
                .setLinearHeadingInterpolation(poseToTaking.getHeading(),take2PosStart.getHeading(),1)
                .build();

//            take2Path = new Path(new BezierLine(take2PosStart,take2PosEnd));
//            take2Path.setConstantHeadingInterpolation(take2PosStart.getHeading());
//        take2Path.setLinearHeadingInterpolation(scorePose.getHeading(),take2PosEnd.getHeading());

        take2Path = follower.pathBuilder()
                .addPath(new BezierLine(take2PosStart,take2PosEnd))
                .setConstantHeadingInterpolation(take2PosStart.getHeading())
                .build();

        take2ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(take2PosEnd,take2PosStart))
                .setLinearHeadingInterpolation(take2PosEnd.getHeading(),take2PosStart.getHeading(),1)

                .addPath(new BezierCurve(take2PosStart,poseToTaking))
                .setLinearHeadingInterpolation(take2PosStart.getHeading(),poseToTaking.getHeading(),1)

                .addPath(new BezierCurve(poseToTaking,scorePose))
                .setLinearHeadingInterpolation(poseToTaking.getHeading(),scorePose.getHeading(),1)
                .build();

        scoreToTake3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,poseToTaking))
                .setLinearHeadingInterpolation(scorePose.getHeading(),poseToTaking.getHeading(),1)

                .addPath(new BezierCurve(poseToTaking,take3PosStart))
                .setLinearHeadingInterpolation(poseToTaking.getHeading(),take3PosStart.getHeading(),1)
                .build();

//            take3Path = new Path(new BezierLine(take3PosStart,take3PosEnd));
//            take3Path.setConstantHeadingInterpolation(take3PosEnd.getHeading());

        take3Path = follower.pathBuilder()
                .addPath(new BezierLine(take3PosStart,take3PosEnd))
                .setConstantHeadingInterpolation(take3PosEnd.getHeading())
                .build();

        take3ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(take3PosEnd,take3PosStart))
                .setLinearHeadingInterpolation(take3PosEnd.getHeading(),take3PosStart.getHeading(),1)

                .addPath(new BezierCurve(take3PosStart,poseToTaking))
                .setLinearHeadingInterpolation(take3PosStart.getHeading(),poseToTaking.getHeading(),1)

                .addPath(new BezierCurve(poseToTaking,scorePose))
                .setLinearHeadingInterpolation(poseToTaking.getHeading(),scorePose.getHeading(),1)
                .build();

        scoreToParking = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,parkingPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),parkingPose.getHeading())
                .build();

    }

    @Override
    public void init(){
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

        isRed = asmConfig.isRed;
        pattern = asmConfig.pattern;

        if(isRed){
            if(!isLongScore){
                scorePose = asmConfig.closeScorePoseRed;
            }else{
                scorePose = asmConfig.longScoreRed;
            }
            poseToTaking = asmConfig.poseToTakingRed;


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

            poseToTaking = asmConfig.poseToTaking;

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

        pathTimer = new Timer();
        pathTimer.resetTimer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        niggTimer = new ElapsedTime();
        niggTimer.reset();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);





        buildPaths();
        telemetry.addData("pattern: ",pattern);
        telemetry.addData("isRed: ",isRed);
        telemetry.update();
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
                if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > timeBetweenShoot){
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
                    if(pattern == 0){
                        follower.followPath(scoreToTake3);
                        pathState = 4;
                    }else if(pattern == 1){
                        follower.followPath(scoreToTake2);
                        pathState = 4;
                    }else if(pattern == 2){
                        follower.followPath(scoreToTake1);
                        pathState = 4;
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    if(pattern == 0){
                        follower.followPath(take3Path);
                        pathState = 5;
                    }else if(pattern == 1){
                        follower.followPath(take2Path);
                        pathState = 5;
                    }else if(pattern == 2){
                        follower.followPath(take1Path);
                        pathState = 5;
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    if(pattern == 0){
                        follower.followPath(take3ToScore);
                        pathState = 6;
                    }else if(pattern == 1){
                        follower.followPath(take2ToScore);
                        pathState = 6;
                    }else if(pattern == 2){
                        follower.followPath(take1toScore);
                        pathState = 6;
                    }
                    niggantroller.toShoot(true);
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
                if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > timeBetweenShoot){
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
                if(!follower.isBusy()){ //========================================================
                    if(pattern == 0){
                        follower.followPath(scoreToTake1);
                        pathState = 9;
                    }else if(pattern == 1){
                        follower.followPath(scoreToTake1);
                        pathState = 9;
                    }else if(pattern == 2){
                        follower.followPath(scoreToTake2);
                        pathState = 9;
                    }
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    if(pattern == 0){
                        follower.followPath(take1Path);
                        pathState = 10;
                    }else if(pattern == 1){
                        follower.followPath(take1Path);
                        pathState = 10;
                    }else if(pattern == 2){
                        follower.followPath(take2Path);
                        pathState = 10;
                    }

                }
                break;
            case 10:
                if(!follower.isBusy()){
                    if(pattern == 0){
                        follower.followPath(take1toScore);
                    }else if(pattern == 1){
                        follower.followPath(take1toScore);
                    }else if(pattern == 2){
                        follower.followPath(take2ToScore);
                    }
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
                if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > timeBetweenShoot){
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
                    if(pattern == 0){
                        follower.followPath(scoreToTake2);
                    }else if(pattern == 1){
                        follower.followPath(scoreToTake3);
                    }else if(pattern == 2){
                        follower.followPath(scoreToTake3);
                    }
                    pathState = 14;
                }
                break;
            case 14:
                if(!follower.isBusy()){
                    if(pattern == 0){
                        follower.followPath(take2Path);
                    }else if(pattern == 1){
                        follower.followPath(take3Path);
                    }else if(pattern == 2){
                        follower.followPath(take3Path);
                    }
                    pathState = 15;
                }
                break;
            case 15:
                if(!follower.isBusy()){
                    if(pattern == 0){
                        follower.followPath(take2ToScore);
                    }else if(pattern == 1){
                        follower.followPath(take3ToScore);
                    }else if(pattern == 2){
                        follower.followPath(take3ToScore);
                    } 
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
                if(niggantroller.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > timeBetweenShoot){
                    if(ballsCount <3){
                        niggantroller.shootBall();
                        niggTimer.reset();
                        ballsCount+=1;
                    }else{
                        niggantroller.toShoot(false);
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

        telemetry.addData("path state", pathState);
        telemetry.addData("balls count",ballsCount);
        telemetry.addData("pattern",pattern);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("target - offset", targetVelocityToCheck - offset);
        niggantroller.showShooterTelemetry(telemetry);
        telemetry.addData("true false:", niggantroller.checkShooterVelocity(targetVelocityToCheck,offset));




        telemetry.update();
    }
}
