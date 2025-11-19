package org.firstinspires.ftc.teamcode.pedroPathing.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Niggantroller;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Utils.asmRobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "pupsik 18 + 0",group = "Competition")
public class Pupsik18 extends OpMode {
    private Follower follower;
    private Timer pathTimer,acitionTimer,opModeTimer;
    private Niggantroller niggantroller;
    private asmRobotState robotState = new asmRobotState();;

    private int pathState = 0;

    public static double targetTurretAngle = 1421;

    private final Pose nullPose = new Pose(0,0,0);
    private final Pose startPose = new Pose(5.56,19.21,2.36);

    private final Pose scorePose = new Pose(28.854,-8.513,0);

    private final Pose take1PosePath = new Pose(43.7,1.59,0);

    private final Pose take1PoseStart = new Pose(44.24,-8.513,1.57);
    private final Pose take1PoseFinal = new Pose(44.24,17.8,1.57);

    private final Pose take2PosePath = new Pose(67,1.59,0);

    private final Pose take2PoseStart = new Pose(68.23,-8.513,1.57);
    private final Pose take2PoseFinal = new Pose(68.23,21.8,1.57);

    private final Pose take3PosePath = new Pose(90,1.59,0);

    private final Pose take3PoseStart = new Pose(92,-8.513,1.57);
    private final Pose take3PoseFinal = new Pose(92,21.8,1.57);

    private final Pose parking = new Pose(30,7.5,1.57);





    public PathChain startToScore1,score1ToTake1,take2ToScore2,scoreToTake2,take2ToScore,score2ToTake3,take3ToScore,take1ToScore,scoreToParking;

    public ElapsedTime niggtimer;
    public void buildPaths(){
        startToScore1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
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
                .setLinearHeadingInterpolation(take1PoseFinal.getHeading(),scorePose.getHeading())
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



        take2ToScore = follower.pathBuilder()
                .addPath(new BezierLine(take2PoseFinal,scorePose))
                .setLinearHeadingInterpolation(take2PoseFinal.getHeading(),scorePose.getHeading())
                .build();

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
                .addPath(new BezierCurve(take3PoseFinal,new Pose(62,2),scorePose))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        scoreToParking = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,parking))
                .setLinearHeadingInterpolation(scorePose.getHeading(),parking.getHeading())
                .build();





    }

    @Override
    public void init(){
        niggtimer = new ElapsedTime();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        niggantroller = new Niggantroller(hardwareMap,telemetry);



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
                    niggantroller.toShoot(true);
                }
                break;
            case 100:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    pathState = 1;

                }
                break;
            case 1:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1000){
                    niggtimer.reset();
                    pathState = 2;
                    niggantroller.intakeEpt(0);
                }
                break;
            case 2:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1000){
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
                    niggantroller.toShoot(true);
                }
                break;
            case 101:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1000){
                    niggtimer.reset();
                    pathState = 6;
                    niggantroller.intakeEpt(0);
                }
                break;
            case 6:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1000){
                    follower.followPath(scoreToTake2);
                    niggantroller.toShoot(false);
                    niggantroller.intakeEpt(1);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.followPath(take2ToScore);
                    niggantroller.toShoot(true);
                    pathState = 102;
                }
                break;
            case 102:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    pathState = 8;
                }
                break;
            case 8:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1000){
                    niggtimer.reset();
                    pathState = 9;
                    niggantroller.intakeEpt(0);
                }
                break;
            case 9:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1000){
                    follower.followPath(score2ToTake3);
                    niggantroller.toShoot(false);
                    niggantroller.intakeEpt(1);
                    pathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    follower.followPath(take3ToScore);
                    niggantroller.toShoot(true);
                    pathState = 103;
                }
                break;
            case 103:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    pathState = 11;
                }
                break;
            case 11:
                if(!follower.isBusy() && niggtimer.milliseconds() > 500){
                    niggtimer.reset();
                    pathState = 12;
                    niggantroller.intakeEpt(0);

                }
                break;
            case 12:
                if(!follower.isBusy()&&niggtimer.milliseconds() > 1000){
                    pathState = 13;
                    follower.followPath(scoreToParking);
                    niggantroller.toShoot(false);
                    niggantroller.intakeEpt(1);
                }
                break;


        }
    }
    @Override
    public void loop(){
        follower.update();
        pathUpdate();


        niggantroller.update(gamepad2.back);
        niggantroller.updateTurret(follower.getPose());
        niggantroller.setTurretMode(TurretController.TurretMode.FIELD_TARGET);

        niggantroller.setFieldAngleTarget(targetTurretAngle);
        niggantroller.setRobotRelativeAngle(targetTurretAngle);
        niggantroller.showTurretTelemetry(telemetry);
        niggantroller.setTurretAutoAimEnabled(true);

        robotState.updatePose(follower.getPose());

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }
}
