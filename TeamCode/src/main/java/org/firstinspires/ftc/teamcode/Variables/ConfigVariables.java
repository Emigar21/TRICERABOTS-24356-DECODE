package org.firstinspires.ftc.teamcode.Variables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigVariables {
    //This values are edited in the Dashboard
    //Turret Orientation Values
    //Recommended Camera 0.003

    //Shooter Values
    public static double power = 0;

    //Turret values
    public static double kTurretP = 0;
    public static double kTurretI = 0;

    public static double kTurretD = 0;
    public static double kTurretF = 0;

    //Chassis Distance Values
    public static double kChassisP = 0.002;
    //Recommended ??
    public static double kChassisI = 0.0002;
    public static double kChassisD;
    public static double kChassisF;

    //Chassis Angle Values
    public static double kAngleChassisP = 0.002;
    public static double kAngleChassisI = 0.0002;
    public static double kAngleChassisD;
    public static double kAngleChassisF;



}
