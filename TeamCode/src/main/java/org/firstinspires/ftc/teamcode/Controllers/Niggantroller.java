package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;

@Config
public class Niggantroller {
    private double intakeDirection = 0;
    private IntakeController intakeController;
    private ShooterControllerPIDVSA shooterController;
    private CommandScheduler outtakeScheduler;
    private TurretController turretController;


    private double intakeStage = -2;
    public static double TIME_BETWEEN_SHOOT = 0.18;
    public static double TIME_BETWEEN_SHOOT_AUTO = 0.18;



    public Niggantroller(HardwareMap hardwareMap,Telemetry newTelemetry){
        intakeController = new IntakeController();
        shooterController = new ShooterControllerPIDVSA();
        outtakeScheduler = new CommandScheduler();
        turretController = new TurretController();

        intakeController.initialize(hardwareMap,"intake_1","intake_2");
        shooterController.initialize(hardwareMap,"shooter_l","shooter_r","l_angle","r_angle",ShooterControllerPIDVSA.ServosPos.DIRECTION_UP.getPos());
        turretController.initialize(hardwareMap,"turret_r");

//        shooterController.setAutoTuningEnabled(true);
    }

    private boolean isShooting = false;

    public static double shooterSpeed = asmConfig.motorVelocityClose;



    public void update(boolean isBack){


        turretController.update();
        outtakeScheduler.update();
        shooterController.update();

//        if (isShooting) {
//            shooterController.setShooterVelocity(shooterSpeed);
//        }

    }

    // -2 stop intake
    // -1 bluet
    // 0 toUp
    // 1 piet

    public void showTelemetry(Telemetry telemetry){
        telemetry.addData("time b shoot",TIME_BETWEEN_SHOOT);
        telemetry.update();
    }
    public void intakeEpt(double intakeState){

        if(intakeStage == -2 && intakeState == -1){
            intakeStage = intakeState;
        }else if( intakeStage == -1 && intakeState == -1){
            intakeStage = -2;
        }
        else if(intakeStage == -2 && intakeState == 0){
            intakeStage = intakeState;
        }else if( intakeStage == 0 && intakeState == 0){
            intakeStage = -2;
        }
        else if(intakeStage == -2 && intakeState == 1){
            intakeStage = intakeState;
        }else if( intakeStage == 1 && intakeState == 1){
            intakeStage = -2;
        }else{
            intakeStage = intakeState;
        }



        if(intakeStage == -1){
            intakeController.setIntakePower(-1);
            intakeController.setSecIntakeMotor(1);

        }else if(intakeStage == 0){
            intakeController.setIntakePower(1);
            intakeController.setSecIntakeMotor(-1);
        }else if(intakeStage == 1){
            intakeController.setIntakePower(1);
            intakeController.setSecIntakeMotor(1);
        }else{
            intakeController.setIntakePower(0);
            intakeController.setSecIntakeMotor(0);
        }
    }

    public void setDirectionPos(double pos){
        shooterController.setDirectionPos(pos);
    }

    public void toShoot(){
        isShooting = !isShooting;
        if(isShooting){
            shooterController.setShooterVelocity(shooterSpeed);
        }else{
            shooterController.setShooterVelocity(0);
        }
    }
    public void shootBall(){
        outtakeScheduler.clearQueue();
        outtakeScheduler.setAutoReset(false);

        outtakeScheduler.scheduleCommand(()-> intakeEpt(0));
        outtakeScheduler.scheduleDelay(TIME_BETWEEN_SHOOT);
        outtakeScheduler.scheduleCommand(()-> intakeEpt(0));

        outtakeScheduler.scheduleCommand(()-> intakeEpt(1));

        outtakeScheduler.start();
    }
    public void toShoot(boolean isShoot){
        isShooting = isShoot;
        if(isShooting){
            shooterController.setShooterVelocity(shooterSpeed);
        }else{
            shooterController.setShooterVelocity(0);
        }
    }


    public void showShooterTelemetry(Telemetry telemetry){
        shooterController.showTelemetry(telemetry);
    }

    public boolean checkShooterVelocity(double targetVelocity,double offset){
        return shooterController.checkVelocity(targetVelocity,offset);
    }

    public void setShooterVelocity(double velocity){
        shooterSpeed = velocity;
    }
    public void setShooterCalibration(boolean isCalibrated){
//        shooterController.setCalibrated(isCalibrated);
    }

    public void setTurretPose(double pose){
        turretController.setTurretPose(pose);
    }



//    public void toTakeSpecimen(){
//
//        outtakeScheduler.clearQueue();
//        outtakeScheduler.setAutoReset(false);
//
//        outtakeScheduler.scheduleCommand(()->  liftController.setTargetPosition(LiftController.Position.SPECIMEN_TAKE.getPos()));
//        outtakeScheduler.scheduleCommand(outtakeController::setOuttakeToTake);
//        outtakeScheduler.scheduleCommand(outtakeController::setClawRotateToTake);
//        outtakeScheduler.scheduleCommand(outtakeController::setClawOpen);
//
//        outtakeScheduler.start();
//
//    }
}
