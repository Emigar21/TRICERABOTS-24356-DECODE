package org.firstinspires.ftc.teamcode.RobotMode.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;
import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;

@Autonomous(name="Testeo odometria", group="neutral")
public class OdometryTesting extends LinearOpMode {
    ChassisController chassis;
    Subsystems subsystems;
    Sensors sensors;
    Camera_Detection cameraDetection;
    TelemetryMethods telemetryMethods;
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

        waitForStart(); // Auto Start

        telemetryMethods.TelemetryUpdateCamera(telemetry);
        Dashboard.initDashboard(ChassisController.getDistanceInchesX(),ChassisController.getDistanceInchesY(),0,15);

            telemetryMethods.TelemetryUpdateCamera(telemetry);
            chassis.AutoMovement(-15, -15, 0);
            sleep(1000);
            chassis.AutoMovement(5, 0, 0);

    }
}

