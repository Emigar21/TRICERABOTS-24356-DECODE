package org.firstinspires.ftc.teamcode.Variables;

public class Constants {

    public static final double TILE_LENGHT = 24;

    public static class  chassisConst {
        public static final int TICS_PER_REV = 8192;
        public static final double WHEEL_CIRC_INCH = 2 * Math.PI; // pi times diameter
    }

    public static class shooterConst{
        public static final double minDist = 34.9;
        public static final double maxDist = 175.76;
        public static final double minVel = .5409;
        public static final double maxVel = .7889;
        //175.76 cm a .7889
        //34.9 cm a .5409

        public static final double HDHEX_TICKS_PER_REV = 21;

    }


}
