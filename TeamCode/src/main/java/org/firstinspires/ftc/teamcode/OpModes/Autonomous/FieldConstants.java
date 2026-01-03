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
            public static final Pose CLOSE_GATE_FACED = new Pose(2, 14.42, 1.57);
            public static final Pose FAR = new Pose(0,0,0);
        }

        public static final Pose SCORE_POSE = new Pose(43.844, -17.878, 0);

        public static final Pose SCORE_POSE_GATE_FACED = new Pose(43.844, -17.878, 1.57);

        public static class Take1 {
            public static final Pose START = new Pose(33.24, -16.465, 1.57);
            public static final Pose FINAL = new Pose(43.844, 25.5, 1.57);
        }

        public static class Take2 {
            public static final Pose START = new Pose(65, -19.465, 1.57);
            public static final Pose MID = new Pose(67.87, 24, 1.57);
            public static final Pose FINAL = new Pose(65, 32, 1.57);
        }

        public static class Take2Gate {
            public static final Pose START = new Pose(65, -16.465, 1.57);
            public static final Pose CURVE = new Pose(0,0,0);
            public static final Pose FINAL = new Pose(65.019, -32.8, -1.57);
        }

        public static class Take3 {
            public static final Pose START = new Pose(91, -20.465, 1.57);
            public static final Pose FINAL = new Pose(91, 34, 1.57);
        }

        public static class Take4 {
            public static final Pose PATH = new Pose(83.64, 30.07, 0.45);
            public static final Pose FINAL = new Pose(118.37, 32.429, 0.45);
        }

        public static class Take5 {
            public static final Pose PATH = new Pose(80.64, 29.07, 0.3);
            public static final Pose FINAL = new Pose(118.37, 27.429, 0.3);
        }

        public static final Pose GATE = new Pose(79.65, 32.5, 2.708);
        public static final Pose GATE_1 = new Pose(63.85, 26.4, 1.57);
        public static final Pose GATE_2 = new Pose(74.262, 32.5, 2.917);

        public static final Pose GATE_OPEN = new Pose(58, 27.8, 0);

        public static final Pose GOAL_POSE = new Pose(-7,-35.179,0);
    }










    public static class Blue {
        public static class Start {
            public static final Pose CLOSE = new Pose(-5.93,-11.2,0);
            public static final Pose CLOSE_GATE_FACED = new Pose(2.0404,-14.894,-1.57);
            public static final Pose FAR = new Pose(0,0,0);
        }

        public static final Pose SCORE_POSE = new Pose(40, 11.64, 0);
        public static final Pose SCORE_POSE_GATE_FACED = new Pose(42.686, 17, -1.57);

        public static class Take1 {
            public static final Pose START = new Pose(40, -4.93, -1.57);
            public static final Pose FINAL = new Pose(44.686, -26.6, -1.57);
        }

        public static class Take2 {
            public static final Pose START = new Pose(66, 19.465, -1.57);
//            public static final Pose MID = new Pose(67.87, 24, 1.57);
            public static final Pose FINAL = new Pose(66, -33, -1.57);
        }

        public static class Take2Gate {
            public static final Pose START = new Pose(66.6, 19.465, -1.57);
            public static final Pose START_ZERO_HEADING = new Pose(62.5, -1.53, 0);
            public static final Pose FINAL = new Pose(65.00, -24.8, -1.57);
        }

        public static class Take3 {
            public static final Pose START = new Pose(91.5, 19.465, -1.57);
            public static final Pose FINAL = new Pose(91.5, -34.2, -1.57);
        }

        public static class Take4 {
            public static final Pose PATH = new Pose(83.64, -30.07, -0.45);
            public static final Pose FINAL = new Pose(118.37,-32.429, -0.45);
        }

        public static class Take5 {
            public static final Pose START = new Pose(81.25, -33.631, 0);
            public static final Pose FINAL1 = new Pose(66.07,-33,-0.55);
            public static final Pose FINAL2 = new Pose(103,-33,-0.8);
        }

        public static final Pose GATE = new Pose(78.65, -32.8, -2.808);
        public static final Pose GATE_1 = new Pose(63.3, -25.2, -1.57);
        public static final Pose GATE_2 = new Pose(73.262, -32.5, -2.717);

        public static final Pose GATE_OPEN = new Pose(58, -27.8, 0);

        public static final Pose GOAL_POSE = new Pose(-7,35.179,0);
    }
}