package org.firstinspires.ftc.teamcode.pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
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


@Autonomous(name = "pupsik 18 + 0",group = "Competition")
public class Pupsik18 extends OpMode {
    private Follower follower;
    private Timer pathTimer,acitionTimer,opModeTimer;
    private Niggantroller niggantroller;
    private asmRobotState robotState = new asmRobotState();;

    private int pathState = 0;

    private final Pose nullPose = new Pose(0,0,0);
    private final Pose startPose = new Pose(-5.451,30.344,2.362);

    private final Pose scorePose1 = new Pose(21.26,-0.918,0);
    private final Pose bezierPose1 = new Pose(46.11,-1.033);
    private final Pose takePose1 = new Pose(35.86,40.76,1.5673);

    private final Pose pathToScore2 = new Pose(46.11,-1.033);
//    private final Pose scorePose2 = new Pose(21.26,-0.918,0);
    private final Pose bezierPose2 = new Pose(56.703,4.819);
    private final Pose takePose2 = new Pose(59.18,44.93,1.534);

//    private final Pose scorePose3 = new Pose(21.26,-0.918,0);
    private final Pose bezierPose3 = new Pose(82.440,2.61);
    private final Pose pathToScore3 = new Pose(56.703,4.819);

    private final Pose takePose3 = new Pose(84.470,44.41,1.53);

    private final Pose gatePose = new Pose(-5,30,0);



    public PathChain startToScore1,scoreToTake1,take1ToScore2,scoreToTake2,take2ToScore,score2ToTake3,take3ToScore;

    public ElapsedTime niggtimer;
    public void buildPaths(){
        startToScore1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose1.getHeading(),0.8)
                .build();

        scoreToTake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1,bezierPose1,takePose1))
                .setTangentHeadingInterpolation()
                .build();

        take1ToScore2 = follower.pathBuilder()
                .addPath(new BezierCurve(takePose1,bezierPose1,scorePose1))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        scoreToTake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,pathToScore2))
                .setLinearHeadingInterpolation(scorePose1.getHeading(),pathToScore2.getHeading(),1)
                .addPath(new BezierCurve(pathToScore2,bezierPose2,takePose2))
                .setTangentHeadingInterpolation()
                .build();

        take2ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(takePose2,bezierPose2,pathToScore2))
                .setTangentHeadingInterpolation().setReversed()

                .addPath(new BezierLine(pathToScore2,scorePose1))
                .setLinearHeadingInterpolation(pathToScore2.getHeading(),scorePose1.getHeading(),1)

                .build();

        score2ToTake3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1,pathToScore3))
                .setLinearHeadingInterpolation(scorePose1.getHeading(),pathToScore3.getHeading())

                .addPath(new BezierCurve(pathToScore3,bezierPose3,takePose3))
                .setTangentHeadingInterpolation()
                .build();

        take3ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(takePose3,bezierPose3,pathToScore3))
                .setTangentHeadingInterpolation().setReversed()

                .addPath(new BezierLine(pathToScore3,scorePose1))
                .setLinearHeadingInterpolation(pathToScore3.getHeading(),scorePose1.getHeading())
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
                    pathState = 1;
                }
                break;
            case 1:
                if(!follower.isBusy()){
                    pathState = 2;
                    niggtimer.reset();
                }
                break;
            case 2:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1500){
                    follower.followPath(scoreToTake1);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy() && niggtimer.milliseconds() > 0){
                    follower.followPath(take1ToScore2);
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    pathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1500){
                    follower.followPath(scoreToTake2);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    pathState = 8;
                }
                break;
            case 8:
                if(!follower.isBusy() && niggtimer.milliseconds() > 0){
                    follower.followPath(take2ToScore);
                    pathState = 9;
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    pathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy() && niggtimer.milliseconds() > 0){
                    follower.followPath(score2ToTake3);
                    pathState = 11;

                }
                break;
            case 11:
                if(!follower.isBusy()){
                    follower.followPath(take3ToScore);
                    pathState = 12;
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
        niggantroller.setTurretMode(TurretController.TurretMode.FIELD_ANGLE);
        niggantroller.setFieldAngleTarget(110);
        niggantroller.showTurretTelemetry(telemetry);
        niggantroller.setTurretAutoAimEnabled(true);

        robotState.updatePose(follower.getPose());

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }
}
