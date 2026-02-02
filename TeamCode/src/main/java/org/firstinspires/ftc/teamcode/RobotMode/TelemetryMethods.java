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
    Telemetry telemetry;

    public void InitTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;

        telemetry.addLine("Processors updated");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start Autonomous");
        telemetry.addLine();
        telemetry.update();
    }
    public void TelemetryUpdateCamera() {
        //Giving the telemetry for the FTC Dashboard (the PC "Driver Station")
        // from the array of CameraDetection
        telemetry.addLine();
        telemetry.addData("Bearing ", bearing);
        telemetry.addData("April Tag ", id);
        telemetry.addData("range", range);
        telemetry.addLine();
        telemetry.addData("Inches in X", ChassisController.getDistanceInchesX());
        telemetry.addData("Inches in Y", ChassisController.getDistanceInchesY());
        telemetry.addData("Error X", 15 - ChassisController.getDistanceInchesX());
        telemetry.addData("Error Y", 15 - ChassisController.getDistanceInchesY());
        telemetry.addData("Setpoint X", 15);
        telemetry.addData("Setpoint Y", 15);
        telemetry.addData("angle", ChassisController.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("turnError", ChassisController.turnError);
        telemetry.update();
    }

    public void TelemetryShooter(Telemetry telemetry){
        telemetry.addData("Actual Vel", Shooter.getActualVel());
        telemetry.update();
    }

    public void odometryTelemetry(Telemetry telemetry){
        telemetry.addData("Inches in X", ChassisController.getDistanceInchesX());
        telemetry.addData("Inches in Y", ChassisController.getDistanceInchesY());
        telemetry.addData("Error X", 15 - ChassisController.getDistanceInchesX());
        telemetry.addData("Error Y", 15 - ChassisController.getDistanceInchesY());
        telemetry.addData("Setpoint X", 15);
        telemetry.addData("Setpoint Y", 15);
        telemetry.update();
    }
}
