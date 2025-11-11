package org.firstinspires.ftc.teamcode.pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Auto test",group = "Competition Auto")
public class Autotest extends OpMode {
    private Follower follower;
    private Timer pathTimer,acitionTimer,opModeTimer;

    private int pathState = 1;

    private final Pose startPose = new Pose(-14.516,11.207,-0.7334);
    private final Pose scorePose = new Pose(-14.516,11.207,-0.7334);
    private final Pose take1PosStart = new Pose(-16.698,0.345,Math.toRadians(0));
    private final Pose take1PosEnd = new Pose(-0,-31.200,-Math.toRadians(180));


    public Path take1Path;

    public Path take2Path;

    public ElapsedTime niggtimer;

    public PathChain posTo;

    public PathChain startToTake1;
    public void buildPaths(){
        startToTake1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,take1PosStart))
                .setLinearHeadingInterpolation(startPose.getHeading(),take1PosStart.getHeading(),1)
                .build();

        take1Path = new Path(new BezierLine(take1PosStart,take1PosEnd));
        take1Path.setLinearHeadingInterpolation(take1PosStart.getHeading(),take1PosEnd.getHeading());
        take1Path.setTranslationalConstraint(0.4);

        take2Path = new Path(new BezierLine(take1PosEnd,take1PosStart));
        take2Path.setLinearHeadingInterpolation(take1PosEnd.getHeading(),take1PosStart.getHeading());
        take2Path.setTranslationalConstraint(0.4);


    }

    @Override
    public void init(){
        niggtimer = new ElapsedTime();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(take1PosStart);


        buildPaths();
    }

    public void pathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(startToTake1);
                pathState = 1;
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.activateAllPIDFs();
                    follower.followPath(take1Path,false);
                    pathState = 2;
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    follower.deactivateAllPIDFs();
                    pathState =3;
                    niggtimer.reset();

                }
                break;
            case 3:
                if( niggtimer.milliseconds() > 3000){
                    if(Math.abs(take1PosEnd.getX() - follower.getPose().getX()) > 2 && !follower.isBusy() ){
                        follower.activateAllPIDFs();
                        follower.followPath(take2Path);
                        pathState = 4;
                    }else{
                        pathState = 1;
                    }
                }
                break;
            case 4:

                if(!follower.isBusy() ){
                    follower.breakFollowing();
                    follower.deactivateAllPIDFs();
                    pathState =5;
                    niggtimer.reset();
                }
                break;
            case 5:
                if(niggtimer.milliseconds() > 3000){
                    pathState = 1;
                    niggtimer.reset();
                }



        }
    }
    @Override
    public void loop(){
        follower.update();
        pathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("X",Math.abs(take1PosEnd.getX() - follower.getPose().getX()));
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
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
