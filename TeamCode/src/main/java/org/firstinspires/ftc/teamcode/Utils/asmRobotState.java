package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class asmRobotState {

    public static Pose poseAfterAuto = new Pose(30,7.5,0);

    public void updatePose(Pose robotPose){
        poseAfterAuto = robotPose;
    }

    public Pose getPoseAfterAuto(){
        return poseAfterAuto;
    }
}
