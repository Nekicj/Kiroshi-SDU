package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OchkoController {
    private Servo stopper = null;
    private Servo ramp = null;

    private double stopperOff = 1;
    private double stopperOn = 0.72;

    private double rampOff = 0.7;
    private double rampOn = 0.38;


    private boolean stopperStateOn = true;
    private boolean rampStateOn = true;


    public void initialize(HardwareMap hardwareMap,String stopperHardwareMapName,String rampHardwareMapName,boolean initPose){
        stopper = hardwareMap.get(Servo.class,stopperHardwareMapName);
        ramp = hardwareMap.get(Servo.class,rampHardwareMapName);

        if(initPose){
            stopper.setPosition(stopperOn);
            ramp.setPosition(rampOn);
        }
    }
    public void setStopper(boolean isStop){
        if(isStop){
            stopper.setPosition(stopperOn);

        }else{
            stopper.setPosition(stopperOff);
        }
        stopperStateOn = isStop;
    }

    public void setRamp(boolean isRamping){
        if(isRamping){
            ramp.setPosition(rampOn);

        }else{
            ramp.setPosition(rampOff);
        }
        rampStateOn = isRamping;
    }

}
