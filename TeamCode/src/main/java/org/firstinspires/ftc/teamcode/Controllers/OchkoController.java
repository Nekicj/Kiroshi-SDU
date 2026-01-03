package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OchkoController {
    private Servo stopper = null;
    private Servo ramp = null;

    public static double stopperOff = 0.98;
    public static double stopperOn = 0.86
            ;

    public static double rampOff = 0.7;
    public static double rampOn = 0.42;


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
