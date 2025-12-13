package org.firstinspires.ftc.teamcode.RobotMode.AUTO;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.artifactPos1;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.artifactPos2;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.artifactPos3;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.artifactsObelisk;
import static org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter.actualArtifact;
import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.ftcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter;

@Autonomous(name = "autoprueba", group = "pene")
public class auto extends LinearOpMode {

    Camera_Detection cameraDetection;
    @Override
    public void runOpMode() throws InterruptedException {
        cameraDetection = new Camera_Detection(hardwareMap);
        Sensors sensors = new Sensors(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);



        VoltageCompensator voltageCompensator = new VoltageCompensator(hardwareMap);
        while(opModeInInit()){
            ftcDashboard.sendImage(cameraDetection.streamProcessor.getLastFrame());

            telemetry.addData("pos1", artifactsObelisk[0]);
            telemetry.addData("pos2", artifactsObelisk[1]);
            telemetry.addData("pos3", artifactsObelisk[2]);
            updateTelemetry(telemetry);

            cameraDetection.CameraDetection();
            cameraDetection.getObelisk();



        }

        waitForStart();
        while(opModeIsActive()){
            ftcDashboard.sendImage(cameraDetection.streamProcessor.getLastFrame());
            telemetry.addData("actual artifact", actualArtifact);
            telemetry.addData("Motor Actual Power", shooter.shooterMotor.getPower());
            updateTelemetry(telemetry);
            cameraDetection.CameraDetection();

            shooter.categorizeColor(sensors.getArtifactColor(),Camera_Detection.range);
        }
        shooter.stopMotors();
    }
}
