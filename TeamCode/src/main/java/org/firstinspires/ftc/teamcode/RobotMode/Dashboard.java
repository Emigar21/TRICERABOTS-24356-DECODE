package org.firstinspires.ftc.teamcode.RobotMode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Dashboard {
    public static FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    public static TelemetryPacket packet;
    public  Telemetry dashboardTelemetry = ftcDashboard.getTelemetry();
    //initialize the  dashboard

    public  Canvas canvas;



    public static void initDashboard(double xPosition, double yPosition, double xGoal, double yGoal){
        packet = new TelemetryPacket(false);

        //Import the image
        packet.fieldOverlay()
                .drawImage("/images/decodefield.png",0,0,144,144)
                .setRotation(Math.toRadians(270))
                .setTranslation(-72, 72)
                .setStroke("green")
                .setStrokeWidth(2)
                .strokeLine(xPosition,yPosition,xGoal,yGoal)//new 0,0
                .setFill("blue")
                .fillCircle(xPosition, yPosition, 3);
        //Send the packet to te ftc dashboard//In progress
        ftcDashboard.sendTelemetryPacket(packet);
    }
}
