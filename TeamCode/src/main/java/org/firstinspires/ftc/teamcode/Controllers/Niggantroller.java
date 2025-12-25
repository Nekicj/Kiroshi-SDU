package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.TurretControllers.TurretControllerCRServo;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;

@Config
public class Niggantroller {
    private double intakeDirection = 0;
    private IntakeController intakeController;
    private ShooterControllerPIDVSA shooterController;
    private CommandScheduler outtakeScheduler;
    private TurretControllerCRServo turretController;
    private OchkoController ochkoController;

    public Niggantroller(HardwareMap hardwareMap,Telemetry newTelemetry){
        intakeController = new IntakeController();
        shooterController = new ShooterControllerPIDVSA();
        outtakeScheduler = new CommandScheduler();
//        turretController = new TurretController();
        ochkoController = new OchkoController();

        intakeController.initialize(hardwareMap,"intake");
        shooterController.initialize(hardwareMap,"shooter_l","shooter_r","r_angle",ShooterControllerPIDVSA.servoClose);
        ochkoController.initialize(hardwareMap,"stopper","ramp",true);
//        turretController.initialize(hardwareMap,"encoder","turret_r");

//        shooterController.setAutoTuningEnabled(true);
    }

    private boolean isShooting = false;

    public static double shooterSpeed = asmConfig.motorVelocityClose;

    private double intakeState = 0;



    public void update(boolean isBack){


//        turretController.update();
        outtakeScheduler.update();
        shooterController.update();

//        if (isShooting) {
//            shooterController.setShooterVelocity(shooterSpeed);
//        }

    }



    public void showTelemetry(Telemetry telemetry){

        telemetry.update();
    }
    public void intakeEpt(double intakeState){

        if(this.intakeState == intakeState){
            this.intakeState = 0;
        }else{
            this.intakeState = intakeState;
        }

        intakeController.setIntakePower(this.intakeState);
    }

    public void setDirectionPos(double pos){
        shooterController.setDirectionPos(pos);
    }

    public void toShootShooter(boolean isShoot){
        isShooting = isShoot;
        if(isShooting){
            shooterController.setShooterVelocity(shooterSpeed);
        }else{
            shooterController.setShooterVelocity(0);
        }
    }

    public void shootBall(boolean toShoot){
        if(toShoot){
            intakeController.setIntakePower(-1);
            ochkoController.setStopper(false);
        }else{
            ochkoController.setStopper(true);
        }

    }

    public void setRamp(boolean rampOn){
        if(rampOn){
            ochkoController.setRamp(true);
        }else{
            ochkoController.setRamp(false);
        }
    }
//    161 91


    public void showShooterTelemetry(Telemetry telemetry){
        shooterController.showTelemetry(telemetry);
    }

    public boolean checkShooterVelocity(double targetVelocity,double offset){
        return shooterController.checkVelocity(targetVelocity,offset);
    }

    public void setShooterVelocity(double velocity){
        shooterSpeed = velocity;
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
