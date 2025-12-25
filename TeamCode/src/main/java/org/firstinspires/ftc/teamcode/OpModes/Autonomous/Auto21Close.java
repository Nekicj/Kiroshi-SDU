package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Niggantroller;
import org.firstinspires.ftc.teamcode.Controllers.ShooterControllerPIDVSA;
import org.firstinspires.ftc.teamcode.Controllers.TurretControllers.TurretControllerMotor;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * хэ
 * Я успешнее тебя
 * пока ты копишь на Off-White, пока я трачу на Goyard
 * хэй
 * Я успешнее тебя
 * Ведь ты своим дерьмом не заработал ни рубля
 * Я успешнее тебя
 * Я как Саня Богданов, у меня так много яхт d
 * Я успешнее тебя
 * Все твои кореша хотят попасть в Кузнецкий Сквад (Сквад)
 */

@Autonomous(name="21 джойстик мне в зад")
public class Auto21Close extends OpMode {
    private enum PathStates{
        START(new Pose(0, 72, 0)),

        TO_SCORE_1(new Pose(0,0,0)),
        SCORE_1(new Pose(0,0,0)),

        TAKE_1_GATE(new Pose(0,0,0)),
        TO_SCORE_2(new Pose(0,0,0)),
        SCORE_2(new Pose(0,0,0)),

        TO_GATE_1(new Pose(0,0,0)),
        GATE_1(new Pose(0,0,0)),
        WAIT_GATE_1(new Pose(0,0,0)),
        TO_SCORE_3(new Pose(0,0,0)),
        SCORE_3(new Pose(0,0,0)),

        TO_GATE_2(new Pose(0,0,0)),
        GATE_2(new Pose(0,0,0)),
        WAIT_GATE_2(new Pose(0,0,0)),
        TO_SCORE_4(new Pose(0,0,0)),
        SCORE_4(new Pose(0,0,0)),

        TO_GATE_3(new Pose(0,0,0)),
        GATE_3(new Pose(0,0,0)),
        WAIT_GATE_3(new Pose(0,0,0)),
        TO_SCORE_5(new Pose(0,0,0)),
        SCORE_5(new Pose(0,0,0)),




        PARKING(new Pose(0,0,0)),
        ;

        final Pose finalPose;
        PathStates(Pose finalPose){
            this.finalPose = finalPose;
        }
    }


    private Follower follower;
    private boolean isBlue = false;
    private TurretControllerMotor turretController;
    private Niggantroller niggantroller;
    private ElapsedTime actionTimer;

    private PathChain START_TO_SCORE,SCORE_TO_TAKE2_N_GATE,SCORE_TO_GATE,GATE_GLOKGLOK,GATE_TO_SCORE,SCORE_TO_PARKING,GATE  = null;

    private void buildPaths(){
        if(isBlue){
            follower.setStartingPose(FieldConstants.Blue.Start.CLOSE);

            START_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.Start.CLOSE,FieldConstants.Blue.SCORE_POSE))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Start.CLOSE.getHeading(),FieldConstants.Blue.SCORE_POSE.getHeading(),0.8)
                    .setTValueConstraint(0.9)
                    .setBrakingStrength(0.4)
                    .setBrakingStart(0.7)

                    .build();

            SCORE_TO_TAKE2_N_GATE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE,FieldConstants.Blue.Take2Gate.START))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE.getHeading(),FieldConstants.Blue.Take2Gate.START.getHeading(),0.6)
                    .setTValueConstraint(0.995)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.START,FieldConstants.Blue.Take2.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.START.getHeading(),FieldConstants.Blue.Take2.FINAL.getHeading(),0.8)
                    .setTValueConstraint(0.85)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2.FINAL,FieldConstants.Blue.Take2Gate.START_ZERO_HEADING))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2.FINAL.getHeading(),FieldConstants.Blue.Take2Gate.START_ZERO_HEADING.getHeading(),0.8)
                    .setTValueConstraint(0.85)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.START_ZERO_HEADING,FieldConstants.Blue.SCORE_POSE))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.START_ZERO_HEADING.getHeading(),FieldConstants.Blue.SCORE_POSE.getHeading(),0.7)
                    .setTValueConstraint(0.85)
                    .setBrakingStrength(0.4)
                    .setBrakingStart(0.6)

                    .build();

            SCORE_TO_GATE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE,FieldConstants.Blue.Take2Gate.START))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE.getHeading(),FieldConstants.Blue.Take2Gate.START.getHeading(),0.8)
                    .setTValueConstraint(0.9)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.START,FieldConstants.Blue.GATE))
//                    .setConstantHeadingInterpolation(FieldConstants.Blue.GATE.getHeading())
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.START.getHeading(),FieldConstants.Blue.GATE.getHeading(),0.7)
                    .setTValueConstraint(0.8)
                    .setBrakingStart(0.6)
                    .setBrakingStrength(0.5)

                    .build();

//            GATE = follower.pathBuilder()
//                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.FINAL,FieldConstants.Blue.GATE))
//                    .setConstantHeadingInterpolation(FieldConstants.Blue.GATE.getHeading())
//                    .setTValueConstraint(0.9)
//                    .setBrakingStart(0.6)
//                    .setBrakingStrength(0.5)
//                    .build();

            GATE_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine( FieldConstants.Blue.GATE,FieldConstants.Blue.Take2Gate.START_ZERO_HEADING))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.GATE.getHeading(),FieldConstants.Blue.Take2Gate.START_ZERO_HEADING.getHeading())
                    .setTValueConstraint(0.9)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.START_ZERO_HEADING,FieldConstants.Blue.SCORE_POSE))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.START_ZERO_HEADING.getHeading(),FieldConstants.Blue.SCORE_POSE.getHeading(),0.7)
                    .setBrakingStrength(0.4)
                    .setBrakingStart(0.6)
                    .build();

            SCORE_TO_PARKING = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE,FieldConstants.Blue.Take2.START))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE.getHeading(),FieldConstants.Blue.Take2.START.getHeading())
                    .build();

        }
    }

    private void pathUpdate(){
        switch (pathState){
            case START:
                if(!follower.isBusy()){
                    follower.followPath(START_TO_SCORE);
                    setPathState(PathStates.TO_SCORE_1);
                    niggantroller.intakeEpt(-1);
                    niggantroller.toShootShooter(true);
                }
                break;
            case TO_SCORE_1:
                if(!follower.isBusy()){
                    actionTimer.reset();
                    setPathState(PathStates.SCORE_1);
                    niggantroller.shootBall(true);
                }
                break;
            case SCORE_1:
                if(!follower.isBusy() && actionTimer.milliseconds() > 900){
                    follower.followPath(SCORE_TO_TAKE2_N_GATE);
                    setPathState(PathStates.TAKE_1_GATE);
                    niggantroller.shootBall(false);
//                    niggantroller.toShootShooter(false);
                }
                break;
            case TAKE_1_GATE:
                if(!follower.isBusy()){
                    setPathState(PathStates.SCORE_2);
                    niggantroller.shootBall(true);
                }
                break;




            case SCORE_2:
                if(!follower.isBusy() && actionTimer.milliseconds() > 900){
                    follower.followPath(SCORE_TO_GATE);
                    setPathState(PathStates.GATE_1);
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                }
                break;

            case GATE_1:
                if(!follower.isBusy()){
//                    follower.setMaxPower(0.8);
//                    follower.followPath(GATE);
                    setPathState(PathStates.WAIT_GATE_1);
                }
                break;
//            case TO_GATE_1:
//                if(!follower.isBusy()){
//                    setPathState(PathStates.WAIT_GATE_1);
////                    follower.setMaxPower(1);
//                }
//                break;
            case WAIT_GATE_1:
                if(!follower.isBusy() && actionTimer.milliseconds() > 1300){
                    follower.followPath(GATE_TO_SCORE);
                    setPathState(PathStates.TO_SCORE_3);
                    niggantroller.toShootShooter(true);
                }
                break;
            case TO_SCORE_3:
                if(!follower.isBusy()){
                    setPathState(PathStates.SCORE_3);
                    niggantroller.shootBall(true);
                }
                break;






            case SCORE_3:
                if(!follower.isBusy() && actionTimer.milliseconds() > 900){
                    follower.followPath(SCORE_TO_GATE);
                    setPathState(PathStates.GATE_2);
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                }
                break;



            case GATE_2:
                if(!follower.isBusy()){
//                    follower.setMaxPower(0.8);
//                    follower.followPath(GATE);
                    setPathState(PathStates.WAIT_GATE_2);
                }
                break;
//            case TO_GATE_2:
//                if(!follower.isBusy()){
//                    follower.setMaxPower(1);
//                    setPathState(PathStates.WAIT_GATE_2);
//                }
//                break;
            case WAIT_GATE_2:
                if(!follower.isBusy() && actionTimer.milliseconds() > 1300){
                    follower.followPath(GATE_TO_SCORE);
                    setPathState(PathStates.TO_SCORE_4);
                    niggantroller.toShootShooter(true);
                }
                break;


            case TO_SCORE_4:
                if(!follower.isBusy()){
                    setPathState(PathStates.SCORE_4);
                    niggantroller.shootBall(true);
                }
                break;




            case SCORE_4:
                if(!follower.isBusy() && actionTimer.milliseconds() > 900){
                    follower.followPath(SCORE_TO_GATE);
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                    setPathState(PathStates.GATE_3);
                }
                break;


            case GATE_3:
                if(!follower.isBusy()){
//                    follower.setMaxPower(0.8);
//                    follower.followPath(GATE);
                    setPathState(PathStates.WAIT_GATE_3);
                }
                break;
//            case TO_GATE_3:
//                if(!follower.isBusy()){
//                    follower.setMaxPower(1);
//                    setPathState(PathStates.WAIT_GATE_3);
//                }
//                break;
            case WAIT_GATE_3:
                if(!follower.isBusy() && actionTimer.milliseconds() > 1300){
                    follower.followPath(GATE_TO_SCORE);
                    setPathState(PathStates.TO_SCORE_5);
                    niggantroller.toShootShooter(true);
                }
                break;

            case TO_SCORE_5:
                if(!follower.isBusy()){
                    setPathState(PathStates.SCORE_5);
                    niggantroller.shootBall(true);
                }
                break;



            case SCORE_5:
                if(!follower.isBusy() && actionTimer.milliseconds() > 1000){
                    follower.followPath(SCORE_TO_PARKING);
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                    setPathState(PathStates.PARKING);
                }
                break;
        }
    }

    PathStates pathState = PathStates.START;
    private void setPathState(PathStates state){
        pathState = state;
        actionTimer.reset();
    }


    @Override
    public void init(){
        isBlue = asmConfig.isBlue;

        actionTimer = new ElapsedTime();
        actionTimer.reset();
        setPathState(PathStates.START);

        turretController = new TurretControllerMotor();
        turretController.initialize(hardwareMap,"turret");
        turretController.setTurretMode(TurretControllerMotor.TurretMode.FIELD_ANGLE);

        niggantroller = new Niggantroller(hardwareMap,telemetry);

        follower = Constants.createFollower(hardwareMap);
        if(isBlue){
            follower.setStartingPose(FieldConstants.Blue.Start.CLOSE);
            turretController.setTargetPoint(FieldConstants.Blue.GOAL_POSE.getX(),FieldConstants.Blue.GOAL_POSE.getY());
            turretController.setFieldAngleTarget(225);
        }else{
            follower.setStartingPose(FieldConstants.Red.Start.CLOSE);
            turretController.setTargetPoint(FieldConstants.Red.GOAL_POSE.getX(),FieldConstants.Red.GOAL_POSE.getY());
        }

        niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoClose);
//        turretController.setFieldAngleTarget(asmConfig.targetTurretCloseFieldCentric);


        niggantroller.setShooterVelocity(asmConfig.motorVelocityClose);





        buildPaths();
        telemetry.addData("isBlue",isBlue);

    }

    @Override
    public void loop(){
        pathUpdate();


        follower.update();
        niggantroller.update(false);
        turretController.update(follower.getPose());
        turretController.showTelemetry(telemetry);
        telemetry.addData("x: ",follower.getPose().getX());
        telemetry.addData("y: ",follower.getPose().getY());
        telemetry.addData("heading: ",follower.getPose().getHeading());
        telemetry.update();
    }
}