package org.firstinspires.ftc.teamcode.ControlSystems;

import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kCompensator;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageCompensator {
    public static VoltageSensor voltageSensor;



    public  VoltageCompensator(HardwareMap hardwareMap){
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public static double getVoltage(){
        return voltageSensor.getVoltage();
    }

    public static double compensateVoltage( double output){
        return output * (kCompensator / getVoltage());
        //12.8
        //.6
        //.75
    }
}
