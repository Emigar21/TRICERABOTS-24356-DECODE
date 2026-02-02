package org.firstinspires.ftc.teamcode.Variables;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {

    public static final double TILE_LENGHT = 24;

    public static class  chassisConst {
        public static final int TICKS_PER_REV = 8192;
        public static final double WHEEL_CIRC_INCH = 2 * Math.PI; // pi times diameter
        public static final double FOLLOW_P = 1;
        public static double CORRECTION_P = -.6;
        public static double CORRECTION_I = 0;
        public static double CORRECTION_D = 0.001;
        public static double CORRECTION_F = 0;
        public static double TURN_P = -.028;
        public static double TURN_I = 0;
        public static double TURN_D = -0.002;
        public static double TURN_F = 0;
    }

    public static class shooterConst {
//        public static final double minDist = 75;
//        public static final double maxDist = 211;
//        public static final double minVel = .51; // .545 - .53 - .52
//        public static final double maxVel = .6985;
        public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(1.3,0,0,4.28);
        public static final double MIN_DISTANCE = 66;
        public static final double MAX_DISTANCE = 320;
        public static final double HDHEX_TICKS_PER_REV = 21;

    }


}
