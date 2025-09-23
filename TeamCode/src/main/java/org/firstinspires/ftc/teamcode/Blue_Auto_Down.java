package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue_Down",group = "Robot Autos")

public class Blue_Auto_Down extends LinearOpMode {

    ChassisController chassis;

    @Override
    public void runOpMode() throws InterruptedException {

        chassis = new ChassisController(hardwareMap);
        Dashboard dashboard = new Dashboard();

        waitForStart();
        ChassisController.startPositionX = Constants.TILE_LENGHT_CM * 2.5;
        ChassisController.startPositionY = 12;
        dashboard.start(); // start the dashboard in a thread
        ChassisController.resetEncoders(); // reset encoders


        while(opModeIsActive()) {
        }

    }
}