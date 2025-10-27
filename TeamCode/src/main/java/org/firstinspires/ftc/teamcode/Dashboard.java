package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;

public class Dashboard {
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    //initialize dashboard

    static TelemetryPacket telemetry;

    //false means that the telemetry packet is not putting the default field

    private static final Canvas DEFAULT_FIELD = new Canvas();

//    static Canvas canvas = packet.fieldOverlay();
    //give the value of the packet to canvas

    static Telemetry dashboardTelemetry = dashboard.getTelemetry();
    //initialize the telemetry



    public static void RunDashboard(){
        telemetry = new TelemetryPacket(false);
        //Import the image
        telemetry.fieldOverlay()
                .drawImage("/dash/decode.webp",0,0,144,144);
//                .setRotation(Math.toRadians(270))//In progress
//                .setFill("blue")//Set color
//                .fillCircle(actualXDistance,actualYDistance , 5)//Make circle.setTranslation(10, actualDistance);
//                .setStroke("red")
//                .strokeLine(actualXDistance,actualYDistance, setPointX, setPointY)
//                .setStroke("green")
//                .strokeLine(initialDistanceX, initialDistanceY, setPointX, setPointY);
//        telemetry.put("Current Distance X", actualXDistance);
//        telemetry.put("Current Distance Y", actualYDistance);
//        telemetry.put("Current Angle", currentAngle);
//        telemetry.put("Error X", XDistance);
//        telemetry.put("Error Y", YDistance);



//        dashboard.sendTelemetryPacket(telemetry);
        //Send the packet to te ftc dashboard
        //dashboard;
    }
}
