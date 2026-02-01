package org.firstinspires.ftc.teamcode.Variables;

public class Constants {

    public static final double TILE_LENGHT = 24;

    public static class  chassisConst {
        public static final int TICKS_PER_REV = 8192;
        public static final double WHEEL_CIRC_INCH = 2 * Math.PI; // pi times diameter
        public static final double FOLLOW_P = 1;

        public static double kChassisP = 0.0098;
    }

    public static class shooterConst {
        public static final double minDist = 75;
        public static final double maxDist = 211;
        public static final double minVel = .51; // .545 - .53 - .52
        public static final double maxVel = .6985;
        //175.76 cm a .7889
        //34.9 cm a .5409
        public static final double AVG_VOLTAGE = 12.8;
        public static final double HDHEX_TICKS_PER_REV = 21;

    }


}
