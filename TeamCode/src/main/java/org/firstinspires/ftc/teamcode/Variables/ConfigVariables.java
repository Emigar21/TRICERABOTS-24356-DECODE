package org.firstinspires.ftc.teamcode.Variables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigVariables {
    //This values are edited in the Dashboard

    //Turret Orientation Values
    //Recommended Camera 0.003
    public static double kTurretP = 0.002;
    //Recommended Camera 0.0002
    public static double kTurretI = 0.0002;

    public static double kTurretD;
    public static double kTurretF;

    //Chassis Orientation Values
    //Recommended 0.075
    public static double kOrientationP = 0.075;
    //Recommended ??
    public static double kOrientationI = 0.000;

    public static double kOrientationD;
    public static double kOrientationF;

    //Chassis Distance Values
    //Recommended ??
    public static double kChassisP = 0.002;
    //Recommended ??
    public static double kChassisI = 0.0002;

    public static double kChassisD;
    public static double kChassisF;

}
