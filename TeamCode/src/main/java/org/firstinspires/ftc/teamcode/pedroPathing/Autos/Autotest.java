package org.firstinspires.ftc.teamcode.pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Auto test",group = "Competition Auto")
public class Autotest extends OpMode {
    private Follower follower;
    private Timer pathTimer,acitionTimer,opModeTimer;

    private int pathState = 1;

    private final Pose startPose = new Pose(0,0,0);
    private final Pose curve_point = new Pose(30,-10,0);
    private final Pose end_pose = new Pose(40,20,Math.toRadians(90));


    public PathChain bezierCurve = null;
    public PathChain bezier = null;

    public ElapsedTime niggtimer;
    public void buildPaths(){
        bezierCurve = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,curve_point,end_pose))
                .setTangentHeadingInterpolation().build();

        bezier = follower.pathBuilder()
                .addPath(new BezierCurve(end_pose,curve_point,startPose))
                .setTangentHeadingInterpolation().build();


    }

    @Override
    public void init(){
        niggtimer = new ElapsedTime();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


        buildPaths();
    }

    public void pathUpdate(){
        switch (pathState){
            case 0:
                if(!follower.isBusy()){
                    follower.followPath(bezierCurve);
                    pathState = 1;
                }
                break;
            case 1:
                if(!follower.isBusy()){
                    niggtimer.reset();
                    pathState = 2;
                }
                break;
            case 2:
                if(!follower.isBusy() && niggtimer.milliseconds() > 1500){
                    follower.followPath(bezier);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    pathState = 4;
                    niggtimer.reset();
                }
                break;
            case 4:
                if(niggtimer.milliseconds() > 1500){
                    pathState = 0;
                }
                break;


        }
    }
    @Override
    public void loop(){
        follower.update();
        pathUpdate();

        telemetry.addData("path state", pathState);
//        telemetry.addData("X",Math.abs(take1PosEnd.getX() - follower.getPose().getX()));
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("closest heading",follower.getClosestPointHeadingGoal());
//        telemetry.addData("closestPosHeading",follower.getClosestPose().getPose().getHeading());
//        telemetry.addData("closest X",follower.getClosestPose().getPose().getX());
//        telemetry.addData("closest Y",follower.getClosestPose().getPose().getY());
//        telemetry.addData("drive error",follower.getDriveError());
//        telemetry.addData("trans error",follower.getTranslationalError());

        telemetry.update();
    }
}
