package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TurretController {

    private CRServo rightTurretServo = null;
    private double servoPose = 0;

    public static enum ServosPos{
        DIRECTION_DOWN(0.8),
        DIRECTION_UP(0.7);


        private final double position;
        ServosPos(double pos) {this.position = pos;}


        public double getPos() {return position;}

    }

    public void initialize(HardwareMap hardwareMap, String turretRightName){
        rightTurretServo = hardwareMap.get(CRServo.class,turretRightName);
    }

    public void update(){
        rightTurretServo.setPower(servoPose);
    }

    public void setTurretPose(double newPose){
        this.servoPose = newPose;
    }
}
