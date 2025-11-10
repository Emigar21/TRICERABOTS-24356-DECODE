package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Dashboard {
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    //initialize dashboard
    static TelemetryPacket packet = new TelemetryPacket(false);



    //false means that the telemetry packet is not putting the default field

    public static Telemetry dashboardTelemetry = dashboard.getTelemetry();
    //initialize the telemetry



    public static void initDashboard(){
        //Import the image
        packet.fieldOverlay()
                .drawImage("/images/decodefield.png",0,0,144,144)
                .setRotation(-Math.toRadians(90))//In progress
                .setTranslation(-72, 72); //new 0,0
        dashboard.sendTelemetryPacket(packet);

    }

    public static void printline(double startX, double startY, double finalX, double finalY){
        packet.fieldOverlay()
                .setStroke("red")
                .setStrokeWidth(1)
                .strokeLine(startX,startY,finalX,finalY);
        dashboard.sendTelemetryPacket(packet);
    }

    public static void moveCircle(double currentPosX, double currentPosY){
        packet.fieldOverlay()
                .setFill("green")
                .fillCircle(currentPosX,currentPosY,5);
        dashboard.sendTelemetryPacket(packet);

    }
}
