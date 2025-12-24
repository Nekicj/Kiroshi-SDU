package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class FieldConstants {

    public static double SHOOTER_VELOCITY_CLOSE = 1150;
    public static double SHOOTER_VELOCITY_CLOSE_BLUE = 1110;
    public static double SHOOTER_VELOCITY_FAR = 1520;

    public static double TURRET_ANGLE_CLOSE = 128;
    public static double TURRET_ANGLE_CLOSE_BLUE = -134; // -128 - 6
    public static double TURRET_ANGLE_FAR = -25;

    public static class Red {
        public static class Start {
            public static final Pose CLOSE = new Pose(5.56, 19.21, 2.36);
            public static final Pose FAR = new Pose(0,0,0);
        }

        public static final Pose SCORE_POSE = new Pose(28.854, -8.513, 0);

        public static class Take1 {
            public static final Pose START = new Pose(37.24, -8.513, 1.57);
            public static final Pose FINAL = new Pose(37.24, 21.8, 1.57);
        }

        public static class Take2 {
            public static final Pose START = new Pose(68.23, -8.513, 1.57);
            public static final Pose FINAL = new Pose(68.23, 28.8, 1.57);
        }

        public static class Take2Gate {
            public static final Pose START = new Pose(65.019, -1.9428, -1.57);
            public static final Pose FINAL = new Pose(65.019, -32.8, -1.57);
        }

        public static class Take3 {
            public static final Pose START = new Pose(92, -8.513, 1.57);
            public static final Pose FINAL = new Pose(92, 27.8, 1.57);
        }

        public static class Take5 {
            public static final Pose PATH = new Pose(80.64, 29.07, 0);
            public static final Pose FINAL = new Pose(118.37, 27.429, 0.3);
        }

        public static final Pose GATE = new Pose(56.642, 23.719, 0);
        public static final Pose SHOOT_POSE_5 = new Pose(115.802, -24.434, -3.16);
    }

    public static class Blue {
        public static class Start {
            public static final Pose CLOSE = new Pose(2.224, -23.19, -2.30);
            public static final Pose FAR = new Pose(/* позиция для дальнего */);
        }

        public static final Pose SCORE_POSE = new Pose(26.838, 5.141, 0);

        public static class Take1 {
            public static final Pose START = new Pose(38.61, -3.483, -1.57);
            public static final Pose FINAL = new Pose(38.61, -23.8, -1.57);
        }

        public static class Take2 {
            public static final Pose START = new Pose(65.019, -1.9428, -1.57);
            public static final Pose FINAL = new Pose(65.019, -32.8, -1.57);
        }

        public static class Take2Gate {
            public static final Pose START = new Pose(65.019, -1.9428, -1.57);
            public static final Pose FINAL = new Pose(65.019, -32.8, -1.57);
        }

        public static class Take3 {
            public static final Pose START = new Pose(84.9, -1.78, -1.57);
            public static final Pose FINAL = new Pose(84.9, -32, -1.57);
        }

        public static class Take5 {
            public static final Pose PATH = new Pose(81.25, -33.631, 0);
            public static final Pose FINAL = new Pose(114, -35.4, -0.2);
        }

        public static final Pose GATE = new Pose(56, -27.377, 0);
        public static final Pose SHOOT_POSE_5 = new Pose(111.17, 15.15, -3.15);
    }
}