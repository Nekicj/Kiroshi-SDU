package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;

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

    public static double motorVelocityCloseBlue = 1390;
    public static double motorOffsetCloseBlue = 50;

    public static double motorVelocityVeryClose = 1200;
    public static double motorOffsetVeryClose = 50;

    public static double targetTurretLongFieldCentric = 39;
    public static double targetTurretCloseFieldCentric = 39;

    public static double targetTurretLongFieldCentricBlue = -45;
    public static double targetTurretCloseFieldCentricBlue = -45;
}
