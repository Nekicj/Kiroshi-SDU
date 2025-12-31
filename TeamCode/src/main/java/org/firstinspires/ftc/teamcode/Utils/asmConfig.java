package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class asmConfig {

    public static double headingAfterAuto = 0;

    public void setHeading(double heading){
        headingAfterAuto = heading;
    }
    public static int pattern = 0;
    public static boolean isBlue = true;

    public static double motorVelocityClose = 1330;
    public static double motorOffsetClose = 50;

    public static double motorVelocityLong = 1840;
    public static double motorOffsetLong = 50;

    public static double targetTurretLongFieldCentric = 67;
    public static double targetTurretCloseFieldCentric = 39;

    public static double targetTurretLongFieldCentricBlue = -75;
    public static double targetTurretCloseFieldCentricBlue = -45;
}
