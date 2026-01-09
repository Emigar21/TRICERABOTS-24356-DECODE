package org.firstinspires.ftc.teamcode.RobotMode.AUTO;

import static org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController.imu;
import static org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController.stopMotors;
import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.ftcDashboard;
import static org.firstinspires.ftc.teamcode.Variables.Constants.TILE_LENGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;

@Autonomous(name = "autoprueba", group = "pene")
public class auto extends LinearOpMode {

    Camera_Detection cameraDetection;
    ChassisController chassisController;
    Sensors sensors;
    Indexer indexer;

    //Shooter shooter;

AutoThread autoThread;
    Dashboard dashboard;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        cameraDetection = new Camera_Detection(hardwareMap);
        sensors = new Sensors(hardwareMap);

        //shooter = new Shooter(hardwareMap);
        chassisController = new ChassisController(hardwareMap);
        VoltageCompensator voltageCompensator = new VoltageCompensator(hardwareMap);

        intake = new Intake(hardwareMap);

        indexer = new Indexer(hardwareMap);


        dashboard = new Dashboard();

        autoThread = new AutoThread();

        dashboard.start();







        while(opModeInInit()){
            ftcDashboard.sendImage(cameraDetection.streamProcessor.getLastFrame());

//            telemetry.addData("pos1", artifactsObelisk[0]);
//            telemetry.addData("pos2", artifactsObelisk[1]);
//            telemetry.addData("pos3", artifactsObelisk[2]);
            updateTelemetry(telemetry);

            cameraDetection.CameraDetection();
            cameraDetection.getObelisk();


        }

        waitForStart();
        chassisController.resetEncoders();
        imu.resetYaw();


        while(opModeIsActive()){

            ftcDashboard.sendImage(cameraDetection.streamProcessor.getLastFrame());

            //cameraDetection.CameraDetection();
            //shooter.categorizeColor(sensors.getArtifactColor(),Camera_Detection.range);

            chassisController.mecanumDriveAuto(0,TILE_LENGHT*1.784, 1);
            chassisController.rotateChassis(-89);


            indexer.moveIndexer(1);

            chassisController.mecanumDriveAuto(0, 24, 0.65);
            chassisController.mecanumDriveAuto(0, 17, 0.4);

            indexer.moveIndexer(0);


            break;


        }
        stopMotors();
    }
}
