package org.firstinspires.ftc.teamcode.RobotMode;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.bearing;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.id;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ControlSystems.PID;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter;

public class TelemetryMethods {
    PID pid = new PID();

    public void InitTelemetry(Telemetry telemetry){
        telemetry.addLine("Processors updated");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start Autonomous");
        telemetry.addLine();
        telemetry.update();
    }
    public void TelemetryUpdateCamera(Telemetry telemetry) {
        //Giving the telemetry for the FTC Dashboard (the PC "Driver Station")
        // from the array of CameraDetection
        telemetry.addLine();
        telemetry.addData("Bearing ", bearing);
        telemetry.addData("April Tag ", id);
        telemetry.addData("range", range);
        telemetry.addData("Inches in X", ChassisController.getDistanceInchesX());
        telemetry.addData("Inches in Y", ChassisController.getDistanceInchesY());
        telemetry.addData("TopLeft Vel", ChassisController.topLeft.getVelocity());
        telemetry.addData("TopRight Vel", ChassisController.topRight.getVelocity());
        telemetry.addData("RearLeft Vel", ChassisController.rearLeft.getVelocity());
        telemetry.addData("RearRight Vel", ChassisController.rearRight.getVelocity());
        telemetry.update();
    }

    public void TelemetryShooter(Telemetry telemetry){
        telemetry.addData("Actual Vel", Shooter.getActualVel());
        telemetry.update();
    }

    public void telemetrySensors(Telemetry telemetry){
        telemetry.addData("Color", Sensors.getColor());
    }
}
