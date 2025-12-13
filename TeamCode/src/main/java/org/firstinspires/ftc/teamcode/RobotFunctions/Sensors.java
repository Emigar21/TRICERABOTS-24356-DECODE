package org.firstinspires.ftc.teamcode.RobotFunctions;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Sensors {
    public static NormalizedColorSensor colorSensor;

    public Sensors (HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class,"colorSensor");
    }


    public static float getColor(){
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        float hsv1 = hsvValues[0]; //purple 210 - 240 green 120 - 180 - elegido
        float hsv2 = hsvValues[1]; //purple 0.3 - 0.6 green 0.7 - 0.8
        float hsv3 = hsvValues[2]; //descartado , el rango de valores es casi el mismo para ambos colores
        return hsv1 ;

    }

    public static String getArtifactColor (){
        if (getColor() >100 && getColor() < 200){
            return "green";
        } else if (getColor() > 200){
            return "purple";
        } else {
            return "null";
        }
    }

}
