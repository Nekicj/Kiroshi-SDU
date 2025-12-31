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

@Autonomous(name="18 ананасов мне в жопу")
public class Auto18WithoutGate extends OpMode {
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

        TO_GATE(new Pose(0,0,0)),
        TO_TAKING_3(new Pose(0,0,0)),
        TAKING_3(new Pose(0,0,0)),
        TO_SCORE_4(new Pose(0,0,0)),
        SCORE_4(new Pose(0,0,0)),

        TO_FINAL1(new Pose(0,0,0)),
        FINAL1_FINAL2(new Pose(0,0,0)),
        FINAL2_TO_SCORE(new Pose(0,0,0)),
        SCORE_5(new Pose(0,0,0)),

        PARKING(new Pose(0,0,0)),
        SEX(new Pose(0,0,0)),
        ;

        final Pose finalPose;
        PathStates(Pose finalPose){
            this.finalPose = finalPose;
        }
    }

    public static double shootingTime = 900;
    private boolean is6 = false;


    private Follower follower;
    private boolean isBlue = false;
    private TurretControllerMotor turretController;
    private Niggantroller niggantroller;
    private ElapsedTime actionTimer;
    private ElapsedTime secondActionTimer;

    private PathChain START_TO_SCORE,SCORE_TO_TAKE1,TAKE1_TO_SCORE, SCORE_TO_TAKE2,TAKE2_TO_SCORE, SCORE_TO_TAKE3, GATE_TO_SCORE,SCORE_TO_FINAL1,FINAL1_TO_FINAL2,FINAL2_TO_SCORE,SCORE_TO_PARKING = null;

    private void buildPaths(){
        if(isBlue){
            follower.setStartingPose(FieldConstants.Blue.Start.CLOSE);

            START_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.Start.CLOSE,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Start.CLOSE.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.7)
                    .setTValueConstraint(0.6)

                    .build();

            SCORE_TO_TAKE1 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE_GATE_FACED,FieldConstants.Blue.Take1.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Blue.Take1.FINAL.getHeading())
                    .setTValueConstraint(0.9)


                    .build();

            TAKE1_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.Take1.FINAL,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take1.FINAL.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.6)
                    .setTValueConstraint(0.995)
                    .build();

            SCORE_TO_TAKE2 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE_GATE_FACED,FieldConstants.Blue.Take2Gate.START))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Blue.Take2Gate.START.getHeading(),0.6)
                    .setTValueConstraint(0.995)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.START,FieldConstants.Blue.Take2.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.START.getHeading(),FieldConstants.Blue.Take2.FINAL.getHeading(),0.8)
                    .setTValueConstraint(0.85)
                    .build();

            TAKE2_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.Take2.FINAL,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2.FINAL.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.8)
                    .setTValueConstraint(0.995)
                    .build();


            SCORE_TO_TAKE3 = follower.pathBuilder()


                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE_GATE_FACED,FieldConstants.Blue.Take3.START))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Blue.Take3.START.getHeading())
                    .setTValueConstraint(0.9)

                    .addPath(new BezierLine(FieldConstants.Blue.Take3.START,FieldConstants.Blue.Take3.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take3.START.getHeading(),FieldConstants.Blue.Take3.FINAL.getHeading())
                    .setTValueConstraint(0.9)

                    .addPath(new BezierLine(FieldConstants.Blue.Take3.FINAL,FieldConstants.Blue.GATE_OPEN))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take3.FINAL.getHeading(),FieldConstants.Blue.GATE_OPEN.getHeading(),1)


                    .build();



            GATE_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.GATE_OPEN,FieldConstants.Blue.SCORE_POSE))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.GATE_OPEN.getHeading(),FieldConstants.Blue.SCORE_POSE.getHeading())
//                    .setTangentHeadingInterpolation()
                    .setTValueConstraint(0.995)
                    .build();




            // TAKE 4 --------------------------------------------------------

            SCORE_TO_FINAL1 = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE,FieldConstants.Blue.Take5.FINAL1))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE.getHeading(),FieldConstants.Blue.Take5.FINAL1.getHeading())
                    .setTValueConstraint(0.8)
                    .build();

            FINAL1_TO_FINAL2 = follower.pathBuilder()


                    .addPath(new BezierLine(FieldConstants.Blue.Take5.FINAL1,FieldConstants.Blue.Take5.FINAL2))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take5.FINAL1.getHeading(),FieldConstants.Blue.Take5.FINAL2.getHeading(),0.7)
                    .setTValueConstraint(0.8)


                    .build();

            FINAL2_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.Take5.FINAL2,FieldConstants.Blue.SCORE_POSE))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take5.FINAL2.getHeading(),FieldConstants.Blue.SCORE_POSE.getHeading())
                    .setTValueConstraint(0.995)
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
                }
                break;
            case SCORE_2:
                if(!follower.isBusy() && actionTimer.milliseconds() > shootingTime){
                    follower.followPath(SCORE_TO_TAKE2);
                    setPathState(PathStates.TO_SCORE_3);
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                }
                break;
            case TO_SCORE_3:
                if(!follower.isBusy()){
                    setPathState(PathStates.TAKING_2);
                    follower.followPath(TAKE2_TO_SCORE);
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
                    setPathState(PathStates.TO_GATE);
                    follower.followPath(SCORE_TO_TAKE3);
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                }
                break;
            case TO_GATE:
                if(!follower.isBusy()){
                    setPathState(PathStates.TO_TAKING_3);
                }
                break;
            case TO_TAKING_3:
                if(!follower.isBusy() && actionTimer.milliseconds() > 1700){
                    setPathState(PathStates.TAKING_3);
//                    follower.followPath(TAKE3_TO_GATE);
                }
                break;
            case TAKING_3:
                if(!follower.isBusy()){
                    follower.followPath(GATE_TO_SCORE);
                    setPathState(PathStates.TO_SCORE_4);
                    niggantroller.toShootShooter(true);
                }
                break;
            case TO_SCORE_4:
                if(!follower.isBusy()){
//                    if(turretController.isOnTarget()){
//                        setPathState(PathStates.SCORE_4);
//                        niggantroller.shootBall(true);
//                    }
                    setPathState(PathStates.SCORE_4);
                    niggantroller.shootBall(true);
                }
                break;
            case SCORE_4:
                if(!follower.isBusy() && actionTimer.milliseconds() > shootingTime){
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                    setPathState(PathStates.TO_FINAL1);
                    follower.followPath(SCORE_TO_FINAL1);
                }
                break;
            case TO_FINAL1:
                if(!follower.isBusy()){
                    setPathState(PathStates.FINAL1_FINAL2);
                    follower.setMaxPower(0.7);


                    follower.followPath(FINAL1_TO_FINAL2);
                }
                break;
//            case TO_FINAL_2:
//                if(!follower.isBusy() || actionTimer.milliseconds() > 3000){
//                    setPathStateWithoutTimer(PathStates.FINAL1_FINAL2);
//                    secondActionTimer.reset();
//                }
//                break;
            case FINAL1_FINAL2:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(FINAL2_TO_SCORE);
                    setPathState(PathStates.FINAL2_TO_SCORE);
                    niggantroller.toShootShooter(true);
                }
                break;
            case FINAL2_TO_SCORE:
                if(!follower.isBusy()){
//                    if(turretController.isOnTarget()){
//                        setPathState(PathStates.SCORE_5);
//                        niggantroller.shootBall(true);
//                    }
                    setPathState(PathStates.SCORE_5);
                    niggantroller.shootBall(true);
                }
                break;
            case SCORE_5:
                if(!follower.isBusy() && actionTimer.milliseconds() > shootingTime){
                    niggantroller.shootBall(false);
                    niggantroller.toShootShooter(false);
                    if(!is6){
                        setPathState(PathStates.TO_FINAL1);
                        is6 = true;
                    }else{
                        setPathState(PathStates.PARKING);
                    }
                }
                break;
            case PARKING:
                if(!follower.isBusy()){
                    turretController.setTurretMode(TurretControllerMotor.TurretMode.ROBOT_RELATIVE);
//                    turretController.setFieldAngleTarget(270);
                    turretController.setRobotRelativeAngle(0);
                    follower.followPath(SCORE_TO_PARKING);
                    setPathState(PathStates.SEX);
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
            turretController.setFieldAngleTarget(218);
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
        telemetry.addData("state: ",pathState);
        telemetry.update();
    }
}