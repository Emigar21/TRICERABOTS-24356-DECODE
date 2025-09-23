package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;

public class Dashboard extends Thread {
    static FtcDashboard dashboard = FtcDashboard.getInstance();
    //initialize dashboard

    static TelemetryPacket packet = new TelemetryPacket(false);
    //false means that the telemetry packet is not putting the default field

    static Canvas canvas = packet.fieldOverlay();
    //give the value of the packet to canvas

    static Telemetry dashboardTelemetry = dashboard.getTelemetry();
    //initialize the telemetry

    @Override
    public void run() {



        while (isAlive()) {


            packet = new TelemetryPacket(false); //false means that the telemetry packet is not putting the default field
            canvas = packet.fieldOverlay(); //give the value of the packet to canvas


            canvas.drawImage("/dash/decode.webp", 0, 0, 144, 144, 0, 72, 72, false)
                    .setRotation(-Math.toRadians(90))
                    .setTranslation(-72, 72);
            canvas.setFill("blue");
            canvas.fillCircle(ChassisController.getDistanceInchesX(), ChassisController.getDistanceInchesY(), 5);

            //TODO:checar que si imprima bien la cancha

            //here we print the field in the dashboard and put the point 0 of the field in the left bottom corner of the field
            //print a circle in the position of the robot


            //lines in the field
            followLine(ChassisController.startPositionX, ChassisController.startPositionY,36,112);
            followLine(36,112,95, 112);
            followLine(95,112,110,88);
            followLine(110,88,110,12);



            //TELEMETRY
            packet.put("TicksX", ChassisController.getTicksX());
            packet.put("DistanceX", ChassisController.getDistanceInchesX());
            packet.put("RevsX", ChassisController.getRevsX());
            packet.put("TicksY", ChassisController.getTicksY());
            packet.put("DistanceY", ChassisController.getDistanceInchesY());
            packet.put("RevsY", ChassisController.getRevsY());
            packet.put("ErrorX", ChassisController.errorAxisX);
            packet.put("ErrorY", ChassisController.errorAxisY);
            packet.put("Angle", -1 * ChassisController.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            //here we put the telemetry in the dashboard

            dashboard.sendTelemetryPacket(packet);
            //here we send the field and telemetry to the dashboard
        }
    }

    public static void followLine (double startX, double startY, double finalX, double finalY) {
        canvas.setStroke("green");
        canvas.setStrokeWidth(1);
        canvas.strokeLine(startX, startY, finalX ,finalY);
    }

    //this function help us to make lines in the field with 4 numbers, we put the coordinate of the start point and the final point

}
