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
    public static boolean isRed = false;

    public static double motorVelocityClose = 1100;
    public static double motorOffsetClose = 50;

    public static double motorVelocityCloseTeleop = 3500;
    public static double motorOffsetCloseTeleop = 200;

    public static double motorVelocityLong = 1550;
    public static double motorOffsetLong = 100;

    public static Pose scorePose = new Pose(34.801,4.545,-2.35);

    public static Pose startPose = new Pose(12.333,-19.389,-2.34);

    //public static Pose startPose = new Pose(12.333,-19.389,-2.34);

    public static Pose closeScorePose = new Pose(26.554,-6.024,-2.365);
    public static Pose longScore = new Pose(54.5,27.5,-2.3);

    public static Pose poseToTaking = new Pose(36.742,6.650,-1.59);

    public static Pose take1PosStart = new Pose(50.391,11,-1.593);
    public static Pose take1PosEnd = new Pose(50.391,-23.997,-1.59);

    public static Pose take2PosStart = new Pose(74.558,11,-1.59);
    public static Pose take2PosEnd = new Pose(74.558,-21.000,-1.59);

    public static Pose take3PosStart = new Pose(97.472,12,-1.59);
    public static Pose take3PosEnd = new Pose(97.472,-23.997,-1.59);

    public static Pose parkingPose = new Pose(46.840,-21.720,-1.53);

    public static Pose takingCurvePose = new Pose(42.44,16.62,-1.77);

    public static Pose take2ToGate = new Pose(63.26,-12.02,-3.1);

    public static Pose take2Gate = new Pose(62.42,-24.024,-3.08);


    // ==================================RED==============================

    public static Pose scorePoseRed = new Pose(34.801,4.545,-2.35);

    public static Pose startPoseRed = new Pose(11.638,19.714,2.353);

    //public static Pose startPoseRed = new Pose(11.638,19.714,2.353);

    public static  Pose closeScorePoseRed = new Pose(28.886,2.316,2.338);
    public static Pose longScoreRed = new Pose(54.5,27.5,-2.3);

    public static Pose poseToTakingRed = new Pose(34,-5,1.5);

    public static Pose take1PosStartRed = new Pose(49.827,-7.036,1.5);
    public static Pose take1PosEndRed = new Pose(51.782,20.144,1.5);

    public static Pose take2PosStartRed = new Pose(71.9,-7.01,1.5);
    public static Pose take2PosEndRed = new Pose(71.9,17.7,1.5);

    public static Pose take3PosStartRed = new Pose(96.651,-11.09,1.5);
    public static Pose take3PosEndRed = new Pose(96.651,18.6,1.5);

    public static Pose parkingPoseRed = new Pose(46.840,10.720,1.5);

    public static Pose takingCurvePoseRed = new Pose(40.407,-19.49,1.77);

    public static Pose take2ToGateRed = new Pose(63.26,12.02,3.1);

    public static Pose take2GateRed = new Pose(62.42,24.024,3.08);
    // 0 - GPP
    // 1 - PGP
    // 2 - PPG
}
