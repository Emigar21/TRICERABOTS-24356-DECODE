package org.firstinspires.ftc.teamcode.RobotMode.AUTO;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.range;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;
import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;

@Autonomous(name="Testeo odometria", group="Testing")
public class OdometryTesting extends LinearOpMode {
    ChassisController chassis;
    Subsystems subsystems;
    Sensors sensors;
    Camera_Detection cameraDetection;
    public static TelemetryMethods telemetryMethods;
    Dashboard dashboard;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new ChassisController(hardwareMap);
        subsystems = new Subsystems(hardwareMap);

        sensors = new Sensors(hardwareMap);
        cameraDetection = new Camera_Detection(hardwareMap);
        dashboard = new Dashboard();

        telemetryMethods = new TelemetryMethods();

        timer = new ElapsedTime();

        telemetryMethods.InitTelemetry(telemetry);
        cameraDetection.CameraDetectionBlue();

        waitForStart(); // Auto Start

        telemetryMethods.TelemetryUpdateCamera();
        Dashboard.initDashboard(ChassisController.getDistanceInchesX(),ChassisController.getDistanceInchesY(),0,15);

        chassis.AutoMovement(0,-10,0);
        chassis.AutoTurn(-40);
    }
}

