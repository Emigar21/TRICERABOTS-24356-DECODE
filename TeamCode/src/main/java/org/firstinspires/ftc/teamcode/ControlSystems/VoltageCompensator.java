package org.firstinspires.ftc.teamcode.ControlSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.minPower;

public class VoltageCompensator {
    public static VoltageSensor voltageSensor;



    public  VoltageCompensator(HardwareMap hardwareMap){
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public static double getVoltage(){
        return voltageSensor.getVoltage();
    }

    public static double compensateVoltage( double output){
        output = output * 12 / getVoltage();
        output = Math.max(output, minPower);

        return output;
    }
}
