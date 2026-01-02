//package org.firstinspires.ftc.teamcode.OpModes.Autonomous.TestAutos;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Controllers.Niggantroller;
//import org.firstinspires.ftc.teamcode.Controllers.ShooterControllerPIDVSA;
//import org.firstinspires.ftc.teamcode.Controllers.TurretControllers.TurretControllerMotor;
//import org.firstinspires.ftc.teamcode.OpModes.Autonomous.FieldConstants;
//import org.firstinspires.ftc.teamcode.Utils.asmConfig;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
///**
// * хэ
// * Я успешнее тебя
// * пока ты копишь на Off-White, пока я трачу на Goyard
// * хэй
// * Я успешнее тебя
// * Ведь ты своим дерьмом не заработал ни рубля
// * Я успешнее тебя
// * Я как Саня Богданов, у меня так много яхт d
// * Я успешнее тебя
// * Все твои кореша хотят попасть в Кузнецкий Сквад (Сквад)
// */
//
//@Autonomous(name="Gate")
//@Disabled
//public class AutoGateTake extends OpMode {
//    private enum PathStates{
//        START(new Pose(0, 72, 0)),
//
//        TO_SCORE_1(new Pose(0,0,0)),
//        SCORE_1(new Pose(0,0,0)),
//
//        TAKE_1_GATE(new Pose(0,0,0)),
//        TO_SCORE_2(new Pose(0,0,0)),
//        SCORE_2(new Pose(0,0,0)),
//
//        TO_GATE_1(new Pose(0,0,0)),
//        GATE_1_TO_GATE(new Pose(0,0,0)),
//        GATE_1(new Pose(0,0,0)),
//        WAIT_GATE_1(new Pose(0,0,0)),
//        TO_SCORE_3(new Pose(0,0,0)),
//        SCORE_3(new Pose(0,0,0)),
//
//        TO_GATE_2(new Pose(0,0,0)),
//        GATE_2(new Pose(0,0,0)),
//        WAIT_GATE_2(new Pose(0,0,0)),
//        TO_SCORE_4(new Pose(0,0,0)),
//        SCORE_4(new Pose(0,0,0)),
//
//        TO_GATE_3(new Pose(0,0,0)),
//        GATE_3(new Pose(0,0,0)),
//        WAIT_GATE_3(new Pose(0,0,0)),
//        TO_SCORE_5(new Pose(0,0,0)),
//        SCORE_5(new Pose(0,0,0)),
//
//
//
//
//        PARKING(new Pose(0,0,0)),
//        ;
//
//        final Pose finalPose;
//        PathStates(Pose finalPose){
//            this.finalPose = finalPose;
//        }
//    }
//
//
//    private Follower follower;
//    private boolean isBlue = false;
//    private TurretControllerMotor turretController;
//    private Niggantroller niggantroller;
//    private ElapsedTime actionTimer;
//
//    private PathChain START_TO_SCORE,SCORE_TO_TAKE2_N_GATE, SCORE_TO_GATE_1,GATE_1_TO_GATE,GATE_GLOKGLOK,GATE_TO_SCORE,SCORE_TO_PARKING,GATE  = null;
//
//    private void buildPaths(){
//        if(isBlue){
//            START_TO_SCORE = follower.pathBuilder()
//                    .addPath(new BezierLine(FieldConstants.Blue.Start.CLOSE,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
//                    .setLinearHeadingInterpolation(FieldConstants.Blue.Start.CLOSE.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.8)
//                    .setTValueConstraint(0.995)
//
//                    .build();
//
//            SCORE_TO_GATE_1 = follower.pathBuilder()
//                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE_GATE_FACED,FieldConstants.Blue.Take2Gate.START))
//                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Blue.Take2Gate.START.getHeading(),0.8)
//                    .setTValueConstraint(0.995)
//
//                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.START,FieldConstants.Blue.GATE))
////                    .setConstantHeadingInterpolation(FieldConstants.Blue.GATE.getHeading())
//                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.START.getHeading(),FieldConstants.Blue.GATE.getHeading(),0.5)
//                    .setTValueConstraint(0.95)
//
//                    .build();
//
//
//
//            GATE_TO_SCORE = follower.pathBuilder()
//                    .addPath(new BezierLine( FieldConstants.Blue.GATE,FieldConstants.Blue.Take2Gate.START))
//                    .setLinearHeadingInterpolation(FieldConstants.Blue.GATE.getHeading(),FieldConstants.Blue.Take2Gate.START.getHeading())
//                    .setTValueConstraint(0.9)
//
//                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.START,FieldConstants.Blue.SCORE_POSE_GATE_FACED))
//                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.START.getHeading(),FieldConstants.Blue.SCORE_POSE_GATE_FACED.getHeading(),0.7)
//                    .setTValueConstraint(0.995)
//                    .build();
//
//
//        }else{
//            START_TO_SCORE = follower.pathBuilder()
//                    .addPath(new BezierLine(FieldConstants.Red.Start.CLOSE_GATE_FACED,FieldConstants.Red.SCORE_POSE_GATE_FACED))
//                    .setLinearHeadingInterpolation(FieldConstants.Red.Start.CLOSE_GATE_FACED.getHeading(),FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),0.8)
//                    .setTValueConstraint(0.995)
//
//                    .build();
//
//
//            SCORE_TO_GATE_1 = follower.pathBuilder()
//                    .addPath(new BezierLine(FieldConstants.Red.SCORE_POSE_GATE_FACED,FieldConstants.Red.GATE_1))
//                    .setLinearHeadingInterpolation(FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading(),FieldConstants.Red.GATE_1.getHeading(),0.8)
//                    .setTValueConstraint(0.995)
//
//                    .build();
//
//
//            GATE_1_TO_GATE = follower.pathBuilder()
//                    .addPath(new BezierLine(FieldConstants.Red.GATE_1,FieldConstants.Red.GATE))
//                    .setLinearHeadingInterpolation(FieldConstants.Red.GATE_1.getHeading(),FieldConstants.Red.GATE.getHeading())
//                    .build();
//
//
//
//            GATE_TO_SCORE = follower.pathBuilder()
//                    .addPath(new BezierLine( FieldConstants.Red.GATE,FieldConstants.Red.SCORE_POSE_GATE_FACED))
//                    .setLinearHeadingInterpolation(FieldConstants.Red.GATE.getHeading(),FieldConstants.Red.SCORE_POSE_GATE_FACED.getHeading())
//                    .setTValueConstraint(0.995)
//
//                    .build();
//        }
//    }
//
//    private void pathUpdate(){
//        switch (pathState){
//
//            case START:
//                if(!follower.isBusy()){
//                    follower.setMaxPower(1);
//                    follower.followPath(START_TO_SCORE);
//                    setPathState(PathStates.TO_SCORE_1);
//                    niggantroller.intakeEpt(-1);
//                    niggantroller.toShootShooter(true);
//                }
//                break;
//            case TO_SCORE_1:
//                if(!follower.isBusy()){
//                    setPathState(PathStates.SCORE_1);
//                    niggantroller.shootBall(true);
//                }
//                break;
//            case SCORE_1:
//                if(!follower.isBusy() && actionTimer.milliseconds() > 800){
//                    setPathState(PathStates.TO_GATE_1);
//                    follower.followPath(SCORE_TO_GATE_1);
//                    niggantroller.shootBall(false);
//                    niggantroller.toShootShooter(false);
//                }
//                break;
//
//            case TO_GATE_1:
//                if(!follower.isBusy()){
//                    setPathState(PathStates.GATE_1_TO_GATE);
//                    follower.followPath(GATE_1_TO_GATE);
//                    follower.setMaxPower(0.65);
//                }
//                break;
//            case GATE_1_TO_GATE:
//                if(!follower.isBusy()){
//                    setPathState(PathStates.GATE_1);
//                    follower.setMaxPower(1);
//                }
//                break;
//            case GATE_1:
//                if(!follower.isBusy() && actionTimer.milliseconds() > 2500){
//                    follower.followPath(GATE_TO_SCORE);
//                    setPathState(PathStates.TO_SCORE_2);
//                    niggantroller.toShootShooter(true);
//                }
//                break;
//            case TO_SCORE_2:
//                if(!follower.isBusy()){
//                    setPathState(PathStates.SCORE_2);
//                    niggantroller.shootBall(true);
//                }
//                break;
//            case SCORE_2:
//                if(!follower.isBusy() && actionTimer.milliseconds() > 800){
//                    niggantroller.shootBall(false);
//                    niggantroller.toShootShooter(false);
//                    setPathState(PathStates.TO_GATE_1);
//                    follower.followPath(SCORE_TO_GATE_1);
//                }
//                break;
//        }
//    }
//
//    PathStates pathState = PathStates.START;
//    private void setPathState(PathStates state){
//        pathState = state;
//        actionTimer.reset();
//    }
//
//
//    @Override
//    public void init(){
//        isBlue = asmConfig.isBlue;
//
//        turretController = new TurretControllerMotor();
//        turretController.initialize(hardwareMap,"turret");
//        turretController.setTurretMode(TurretControllerMotor.TurretMode.FIELD_ANGLE);
//
//        niggantroller = new Niggantroller(hardwareMap,telemetry);
//
//        follower = Constants.createFollower(hardwareMap);
//
//
//
//
//        if(isBlue){
//            follower.setStartingPose(FieldConstants.Blue.Start.CLOSE_GATE_FACED);
//            follower.setPose(FieldConstants.Blue.Start.CLOSE_GATE_FACED);
//            turretController.setTargetPoint(FieldConstants.Blue.GOAL_POSE.getX(),FieldConstants.Blue.GOAL_POSE.getY());
//            turretController.setFieldAngleTarget(225);
//        }else{
//            follower.setStartingPose(FieldConstants.Red.Start.CLOSE_GATE_FACED);
//            follower.setPose(FieldConstants.Red.Start.CLOSE_GATE_FACED);
//            turretController.setTargetPoint(FieldConstants.Red.GOAL_POSE.getX(),FieldConstants.Red.GOAL_POSE.getY());
//            turretController.setFieldAngleTarget(-225);
//        }
//
//
//        follower.update();
//
//
//        actionTimer = new ElapsedTime();
//
//        niggantroller.setDirectionPos(ShooterControllerPIDVSA.servoClose);
////        turretController.setFieldAngleTarget(asmConfig.targetTurretCloseFieldCentric);
//
//
//        niggantroller.setShooterVelocity(asmConfig.motorVelocityClose);
//
//
//
//
//
//        buildPaths();
//        telemetry.addData("isBlue",isBlue);
//
//    }
//
//    @Override
//    public void loop(){
//        pathUpdate();
//
//
//        follower.update();
//        niggantroller.update(false);
//        niggantroller.showShooterTelemetry(telemetry);
//        turretController.update(follower.getPose());
//        turretController.showTelemetry(telemetry);
//        telemetry.addData("x: ",follower.getPose().getX());
//        telemetry.addData("y: ",follower.getPose().getY());
//        telemetry.addData("heading: ",follower.getPose().getHeading());
//        telemetry.addData("state: ",pathState);
//        telemetry.update();
//    }
//}