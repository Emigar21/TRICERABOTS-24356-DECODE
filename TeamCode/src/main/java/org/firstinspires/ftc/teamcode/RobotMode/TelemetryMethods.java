package org.firstinspires.ftc.teamcode.RobotMode;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.bearing;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.id;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.range;
import static org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Feeder.feederMotor;
import static org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Indexer.indexerMotor;
import static org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Intake.intakeMotor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ControlSystems.PID;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter;

public class TelemetryMethods {
    PID pid = new PID();
    public void ClearTelemetry(Telemetry telemetry){
        telemetry.clearAll();
    }

    public void InitTelemetry(Telemetry telemetry){
        telemetry.addLine("Processors updated");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start Autonomous");
        telemetry.addLine();
        telemetry.update();
    }

    public void TelemetryChassis(Telemetry telemetry){
        telemetry.addData("TopLeft Power", ChassisController.topLeft.getPower());
        telemetry.addData("TopRight Power", ChassisController.topRight.getPower());
        telemetry.addData("rearLeft Power", ChassisController.rearLeft.getPower());
        telemetry.addData("rearRight Power", ChassisController.rearRight.getPower());
    }
    public void TelemetryUpdateCamera(Telemetry telemetry) {
        //Giving the telemetry for the FTC Dashboard (the PC "Driver Station")
        // from the array of CameraDetection
        telemetry.addLine();
        telemetry.addData("Bearing ", bearing);
        telemetry.addData("April Tag ", id);
        telemetry.addData("range", range);
        telemetry.addData("Auto running", "Look for more");
        telemetry.update();
    }

    public void TelemetryShooter(Telemetry telemetry){
        telemetry.addData("Shooter power", Shooter.getShooterPower(range));
        telemetry.addData("Actual Vel", Shooter.getActualVel());
        telemetry.addData("Revs Needed", Shooter.getDesiredRevs());
    }

    public void TelemetryCyclying(Telemetry telemetry){
        telemetry.addData("Intake Revs", intakeMotor.getVelocity());
        telemetry.addData("Indexer Revs", indexerMotor.getVelocity());
        telemetry.addData("Feeder", feederMotor.getVelocity());
    }

    public void telemetrySensors(Telemetry telemetry){
        telemetry.addData("Color", Sensors.getColor());
    }
}
