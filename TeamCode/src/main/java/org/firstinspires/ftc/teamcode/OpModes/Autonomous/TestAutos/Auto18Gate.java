package org.firstinspires.ftc.teamcode.OpModes.Autonomous.TestAutos;

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
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.FieldConstants;

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

@Autonomous(name="6 * 3 = gate")
public class Auto18Gate extends OpMode {
    private enum PathStates{
        START(new Pose(0, 72, 0)),

        TO_SCORE_1(new Pose(0,0,0)),
        SCORE_1(new Pose(0,0,0)),

        TAKING_1(new Pose(0,0,0)),
        TO_SCORE_2(new Pose(0,0,0)),
        SCORE_2(new Pose(0,0,0)),

        TAKING_2(new Pose(0,0,0)),
        TO_SCORE_3(new Pose(0,0,0)),
        SCORE_3(new Pose(0,0,0)),

        TAKE_3_TO_SCORE(new Pose(0,0,0)),
        TO_GATE(new Pose(0,0,0)),
        TO_TAKING_3(new Pose(0,0,0)),
        TAKING_3(new Pose(0,0,0)),
        TO_SCORE_4(new Pose(0,0,0)),
        SCORE_4(new Pose(0,0,0)),

        TO_TAKE_4(new Pose(0,0,0)),

        TO_GATE_1(new Pose(0,0,0)),
        GATE_WAIT(new Pose(0,0,0)),
        GATE_TO_GATE_1(new Pose(0,0,0)),
        GATE(new Pose(0,0,0)),
        GATE_WAIT_2(new Pose(0,0,0)),

        TO_SCORE_5(new Pose(0,0,0)),
        SCORE_5(new Pose(0,0,0)),


        PARKING(new Pose(0,0,0)),

        ;

        final Pose finalPose;
        PathStates(Pose finalPose){
            this.finalPose = finalPose;
        }
    }

    public static double shootingTime = 750;
    private boolean is6 = false;
    private boolean is5 = false;


    private Follower follower;
    private boolean isBlue = false;
    private TurretControllerMotor turretController;
    private Niggantroller niggantroller;
    private ElapsedTime actionTimer;
    private ElapsedTime secondActionTimer;
    private ElapsedTime matchTimer ;

    private PathChain START_TO_SCORE,SCORE_TO_TAKE1,TAKE1_TO_SCORE, SCORE_TO_TAKE2,TAKE2_TO_SCORE, SCORE_TO_TAKE3, TAKE_3_TO_SCORE, GATE_TO_SCORE, SCORE_TO_TAKE4,TAKE_4,TAKE4_TO_SCORE,GATE_TO_GATE_1,SCORE_TO_PARKING,SCORE_TO_GATE_1,GATE_1_TO_GATE,GATE_1_TO_SCORE = null;

    private void buildPaths(){
        if(isBlue){
            START_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.Start.CLOSE_GATE_FACED,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Start.CLOSE_GATE_FACED.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.7)
                    .setTValueConstraint(0.995)

                    .build();


            // =======================================TAKE1=========================================

            SCORE_TO_TAKE1 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE_GATE_FACED,FieldConstants.Blue.Take1.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Blue.Take1.FINAL.getHeading(),0.8)
                    .setTValueConstraint(0.5)


                    .build();

            TAKE1_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.Take1.FINAL,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take1.FINAL.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.6)
                    .setTValueConstraint(0.995)
                    .build();

            // ======================================TAKE2==========================================

            SCORE_TO_TAKE2 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE_GATE_FACED,FieldConstants.Blue.Take2Gate.START))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Blue.Take2Gate.START.getHeading(),0.6)
                    .setTValueConstraint(0.995)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.START,FieldConstants.Blue.Take2.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.START.getHeading(),FieldConstants.Blue.Take2.FINAL.getHeading(),0.8)
                    .setTValueConstraint(0.85)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2.FINAL,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2.FINAL.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.7)
                    .setTValueConstraint(0.6)

                    .build();



//            TAKE2_TO_SCORE = follower.pathBuilder()
//                    .addPath(new BezierLine(FieldConstants.Red.SCORE_POSE,FieldConstants.Red.SCORE_POSE))
//                    .setLinearHeadingInterpolation(FieldConstants.Red.SCORE_POSE.getHeading(),FieldConstants.Red.SCORE_POSE.getHeading(),0.8)
//                    .setTValueConstraint(0.995)
//                    .build();

            // =====================================================================================


            SCORE_TO_GATE_1 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE_GATE_FACED,FieldConstants.Blue.GATE_1))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Blue.GATE_1.getHeading(),0.7)
                    .setTValueConstraint(0.9)
                    .build();

            GATE_1_TO_GATE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.GATE_1,FieldConstants.Blue.GATE))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.GATE_1.getHeading(),FieldConstants.Blue.GATE.getHeading(),0.7)
                    .setTValueConstraint(0.9)

                    .build();

            GATE_TO_GATE_1 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.GATE,FieldConstants.Blue.GATE_2))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.GATE.getHeading(),FieldConstants.Blue.GATE_2.getHeading(),0.7)
                    .setTValueConstraint(0.6)

                    .build();

            GATE_1_TO_SCORE = follower.pathBuilder()

                    .addPath(new BezierLine(FieldConstants.Blue.GATE_2,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.GATE_2.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.7)
                    .setTValueConstraint(0.8)
                    .build();





            // ======================================TAKE3==========================================


            SCORE_TO_TAKE3 = follower.pathBuilder()


                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE_GATE_FACED,FieldConstants.Blue.Take3.START))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Blue.Take3.START.getHeading(),0.7)
                    .setTValueConstraint(0.6)

                    .addPath(new BezierLine(FieldConstants.Blue.Take3.START,FieldConstants.Blue.Take3.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take3.START.getHeading(),FieldConstants.Blue.Take3.FINAL.getHeading(),0.7)
                    .setTValueConstraint(0.7)

                    .addPath(new BezierLine(FieldConstants.Blue.Take3.FINAL,FieldConstants.Blue.GATE))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take3.FINAL.getHeading(),FieldConstants.Blue.GATE.getHeading(),0.7)
                    .setTValueConstraint(0.7)

                    .build();

            TAKE_3_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.GATE_1,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.GATE_1.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.75)
                    .setTValueConstraint(0.8)
                    .build();;


            // =====================================================================================


            // ====================================TAKE4============================================


            SCORE_TO_PARKING = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE_GATE_FACED,FieldConstants.Blue.Take2.START))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Blue.Take2.START.getHeading())
                    .build();
        }else{
            START_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Red.Start.CLOSE_GATE_FACED,FieldConstants.Red.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Red.Start.CLOSE_GATE_FACED.getHeading(),FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),0.7)
                    .setTValueConstraint(0.995)

                    .build();


            // =======================================TAKE1=========================================

            SCORE_TO_TAKE1 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Red.SCORE_POSE_GATE_FACED,FieldConstants.Red.Take1.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Red.Take1.FINAL.getHeading(),0.8)
                    .setTValueConstraint(0.5)


                    .build();

            TAKE1_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Red.Take1.FINAL,FieldConstants.Red.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Red.Take1.FINAL.getHeading(),FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),0.6)
                    .setTValueConstraint(0.995)
                    .build();

            // ======================================TAKE2==========================================

            SCORE_TO_TAKE2 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Red.SCORE_POSE_GATE_FACED,FieldConstants.Red.Take2Gate.START))
                    .setLinearHeadingInterpolation(FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Red.Take2Gate.START.getHeading(),0.6)
                    .setTValueConstraint(0.995)

                    .addPath(new BezierLine(FieldConstants.Red.Take2Gate.START,FieldConstants.Red.Take2.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Red.Take2Gate.START.getHeading(),FieldConstants.Red.Take2.FINAL.getHeading(),0.8)
                    .setTValueConstraint(0.85)

                    .addPath(new BezierLine(FieldConstants.Red.Take2.FINAL,FieldConstants.Red.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Red.Take2.FINAL.getHeading(),FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),0.7)
                    .setTValueConstraint(0.6)

                    .build();



//            TAKE2_TO_SCORE = follower.pathBuilder()
//                    .addPath(new BezierLine(FieldConstants.Red.SCORE_POSE,FieldConstants.Red.SCORE_POSE))
//                    .setLinearHeadingInterpolation(FieldConstants.Red.SCORE_POSE.getHeading(),FieldConstants.Red.SCORE_POSE.getHeading(),0.8)
//                    .setTValueConstraint(0.995)
//                    .build();

            // =====================================================================================


            SCORE_TO_GATE_1 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Red.SCORE_POSE_GATE_FACED,FieldConstants.Red.GATE_1))
                    .setLinearHeadingInterpolation(FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Red.GATE_1.getHeading(),0.7)
                    .setTValueConstraint(0.9)
                    .build();

            GATE_1_TO_GATE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Red.GATE_1,FieldConstants.Red.GATE))
                    .setLinearHeadingInterpolation(FieldConstants.Red.GATE_1.getHeading(),FieldConstants.Red.GATE.getHeading(),0.7)
                    .setTValueConstraint(0.9)

                    .build();

            GATE_TO_GATE_1 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Red.GATE,FieldConstants.Red.GATE_2))
                    .setLinearHeadingInterpolation(FieldConstants.Red.GATE.getHeading(),FieldConstants.Red.GATE_2.getHeading(),0.7)
                    .setTValueConstraint(0.6)

                    .build();

            GATE_1_TO_SCORE = follower.pathBuilder()

                    .addPath(new BezierLine(FieldConstants.Red.GATE_2,FieldConstants.Red.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Red.GATE_2.getHeading(),FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),0.7)
                    .setTValueConstraint(0.8)
                    .build();





            // ======================================TAKE3==========================================


            SCORE_TO_TAKE3 = follower.pathBuilder()


                    .addPath(new BezierLine(FieldConstants.Red.SCORE_POSE_GATE_FACED,FieldConstants.Red.Take3.START))
                    .setLinearHeadingInterpolation(FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Red.Take3.START.getHeading(),0.7)
                    .setTValueConstraint(0.6)

                    .addPath(new BezierLine(FieldConstants.Red.Take3.START,FieldConstants.Red.Take3.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Red.Take3.START.getHeading(),FieldConstants.Red.Take3.FINAL.getHeading(),0.7)
                    .setTValueConstraint(0.7)

                    .addPath(new BezierLine(FieldConstants.Red.Take3.FINAL,FieldConstants.Red.GATE))
                    .setLinearHeadingInterpolation(FieldConstants.Red.Take3.FINAL.getHeading(),FieldConstants.Red.GATE.getHeading(),0.7)
                    .setTValueConstraint(0.7)

                    .build();

            TAKE_3_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Red.Take3.FINAL,FieldConstants.Red.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Red.Take3.FINAL.getHeading(),FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),0.75)
                    .setTValueConstraint(0.8)
                    .build();;


            // =====================================================================================


            // ====================================TAKE4============================================


            SCORE_TO_PARKING = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Red.SCORE_POSE_GATE_FACED,FieldConstants.Red.Take2.START))
                    .setLinearHeadingInterpolation(FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Red.Take2.START.getHeading())
                    .build();




        }
    }

    private void pathUpdate(){
        switch (pathState){
            case START:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(START_TO_SCORE);
                    setPathState(PathStates.TO_SCORE_1);
                    niggantroller.intakeEpt(-1);
                    niggantroller.toShootShooter(true);
                    matchTimer.reset();
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
                if(!follower.isBusy() && actionTimer.milliseconds() > shootingTime){
                    follower.followPath(SCORE_TO_TAKE1);
                    setPathState(PathStates.TAKING_1);
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                }
                break;
            case TAKING_1:
                if(!follower.isBusy()){
                    setPathState(PathStates.TO_SCORE_2);
                    follower.setMaxPower(1);
                    follower.followPath(TAKE1_TO_SCORE);
                    niggantroller.toShootShooter(true);
                }
                break;
            case TO_SCORE_2:
                if(!follower.isBusy()){
//                    if(turretController.isOnTarget()){
//                        setPathState(PathStates.SCORE_2);
//                        niggantroller.shootBall(true);
//                    }
                    setPathState(PathStates.SCORE_2);
                    niggantroller.shootBall(true);
                    follower.setMaxPower(1);
                }
                break;
            case SCORE_2:
                if(!follower.isBusy() && actionTimer.milliseconds() > shootingTime){
                    follower.followPath(SCORE_TO_TAKE2);
                    setPathState(PathStates.TO_GATE);
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                }
                break;
            case TO_GATE:
                if(!follower.isBusy()){
                    setPathState(PathStates.TO_SCORE_3);
                }
                break;
            case TO_SCORE_3:
                if(!follower.isBusy() && actionTimer.milliseconds()>0){
                    setPathState(PathStates.TAKING_2);
//                    follower.followPath(TAKE2_TO_SCORE);
                    niggantroller.toShootShooter(true);
                }
                break;
            case TAKING_2:
                if(!follower.isBusy()){
//                    if(turretController.isOnTarget()){
//                        setPathState(PathStates.SCORE_3);
//                        niggantroller.shootBall(true);
//                    }
                    setPathState(PathStates.SCORE_3);
                    niggantroller.shootBall(true);

                }
                break;

            case SCORE_3:
                if(!follower.isBusy() && actionTimer.milliseconds() > shootingTime){
                    setPathState(PathStates.TO_GATE_1);
                    follower.setMaxPower(1);
                    follower.followPath(SCORE_TO_GATE_1);
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                }
                break;




// GATE =======================================================



            case TO_GATE_1:
                if(!follower.isBusy()){
                    setPathState(PathStates.GATE_WAIT);
                }
                break;
            case GATE_WAIT:
                if(!follower.isBusy() && actionTimer.milliseconds() > 550){
                    follower.followPath(GATE_1_TO_GATE);
                    setPathState(PathStates.GATE_WAIT_2);
                }
                break;
            case GATE_WAIT_2:
                if(!follower.isBusy() || actionTimer.milliseconds() > 900){
                    setPathState(PathStates.GATE);
                }
                break;

            case GATE:
                if(!follower.isBusy() && actionTimer.milliseconds() > 900){
                    setPathState(PathStates.GATE_TO_GATE_1);
                    follower.setMaxPower(0.6);
                    follower.followPath(GATE_TO_GATE_1);

                    niggantroller.toShootShooter(true);
                }
                break;
            case GATE_TO_GATE_1:
                if(!follower.isBusy() || actionTimer.milliseconds() > 1200){
                    follower.followPath(GATE_1_TO_SCORE);
                    follower.setMaxPower(1);
                    setPathState(PathStates.TO_SCORE_5);
                }
                break;




            case TO_SCORE_5:
                if(!follower.isBusy()){
                    setPathState(PathStates.SCORE_5);
                    niggantroller.shootBall(true);
                }
                break;
            case SCORE_5:
                if(!follower.isBusy() && actionTimer.milliseconds() > shootingTime){
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);

                    follower.setMaxPower(1);

                    if(is5){
                        setPathState(PathStates.TO_TAKING_3);
                        follower.followPath(SCORE_TO_TAKE3);
                    }else{
                        setPathState(PathStates.TO_GATE_1);
                        follower.followPath(SCORE_TO_GATE_1);
                    }

                    is5 = true;
                }
                break;






// Take3 =====================================================
            case TO_TAKING_3:
                if(!follower.isBusy()){
                    setPathState(PathStates.TAKE_3_TO_SCORE);
                    follower.setMaxPower(1);
                    follower.followPath(TAKE_3_TO_SCORE);
                    niggantroller.toShootShooter(true);
                }
                break;
            case TAKE_3_TO_SCORE:
                if(!follower.isBusy()){
                    setPathState(PathStates.TAKING_3);
                    niggantroller.shootBall(true);
                }
                break;

            case TAKING_3:
                if(!follower.isBusy() && actionTimer.milliseconds() > shootingTime){
                    follower.followPath(SCORE_TO_PARKING);
                    turretController.setTurretMode(TurretControllerMotor.TurretMode.ROBOT_RELATIVE);
//                    turretController.setFieldAngleTarget(270);
                    turretController.setRobotRelativeAngle(0);;
                    setPathState(PathStates.PARKING);
                    niggantroller.toShootShooter(false);
                    niggantroller.shootBall(false);
                }
                break;








        }

    }

    PathStates pathState = PathStates.START;
    private void setPathState(PathStates state){
        pathState = state;
        actionTimer.reset();
    }
    private void setPathStateWithoutTimer(PathStates state){
        pathState = state;
    }


    @Override
    public void init(){
        isBlue = asmConfig.isBlue;




        turretController = new TurretControllerMotor();
        turretController.initialize(hardwareMap,"turret");
        turretController.setTurretMode(TurretControllerMotor.TurretMode.FIELD_ANGLE);

        niggantroller = new Niggantroller(hardwareMap,telemetry);

        follower = Constants.createFollower(hardwareMap);
        if(isBlue){
            niggantroller.setShooterVelocity(asmConfig.motorVelocityClose);
            follower.setStartingPose(FieldConstants.Blue.Start.CLOSE_GATE_FACED);
            follower.setPose(FieldConstants.Blue.Start.CLOSE_GATE_FACED);
            turretController.setTargetPoint(FieldConstants.Blue.GOAL_POSE.getX(),FieldConstants.Blue.GOAL_POSE.getY());
            turretController.setFieldAngleTarget(225);
        }else{
            niggantroller.setShooterVelocity(asmConfig.motorVelocityClose);
            follower.setStartingPose(FieldConstants.Red.Start.CLOSE_GATE_FACED);
            follower.setPose(FieldConstants.Red.Start.CLOSE_GATE_FACED);
            turretController.setTargetPoint(FieldConstants.Red.GOAL_POSE.getX(),FieldConstants.Red.GOAL_POSE.getY());
            turretController.setFieldAngleTarget(-225);
        }
        follower.update();

        actionTimer = new ElapsedTime();
        matchTimer = new ElapsedTime();
        matchTimer.reset();
        actionTimer.reset();
        setPathState(PathStates.START);

        niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoClose);
//        turretController.setFieldAngleTarget(asmConfig.targetTurretCloseFieldCentric);








        buildPaths();
        telemetry.addData("isBlue",isBlue);
        telemetry.addData("heading: ",follower.getPose().getHeading());
        telemetry.addData("x: ",follower.getPose().getX());
        telemetry.addData("y: ",follower.getPose().getY());


    }

    @Override
    public void loop(){
        pathUpdate();

        if(matchTimer.milliseconds() > 29500){
            turretController.setTurretMode(TurretControllerMotor.TurretMode.ROBOT_RELATIVE);
//                    turretController.setFieldAngleTarget(270);
            turretController.setRobotRelativeAngle(0);
        }
        if(matchTimer.milliseconds() > 30500){
            requestOpModeStop();
        }


        follower.update();
        niggantroller.update(false);
        turretController.update(follower.getCurrentPath().getLastControlPoint().getPose());
        turretController.showTelemetry(telemetry);
        telemetry.addData("x: ",follower.getPose().getX());
        telemetry.addData("y: ",follower.getPose().getY());
        telemetry.addData("heading: ",follower.getPose().getHeading());
        telemetry.addData("state: ",pathState);
        telemetry.update();
    }
}