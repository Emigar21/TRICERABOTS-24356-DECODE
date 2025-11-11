package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Dashboard.dashboardTelemetry;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotFunctions.ChassisController;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

@TeleOp(name="Main",group="Robot")
public class Main extends OpMode {

    ChassisController chassis;

    // Controller Input

    // Gamepad 1
    double LT1, RT1;
    double LSx1, LSy1, RSx1, RSy1;
    boolean LB1, RB1;
    boolean A1,B1,Y1,X1;
    boolean dPadUp1, dPadDown1, dPadRight1, dPadLeft1;

    // Gamepad 2
    float LT2, RT2;
    double LSx2, LSy2, RSx2, RSy2;
    boolean LB2, RB2;
    boolean dPadUp2, dPadDown2, dPadRight2, dPadLeft2;
    boolean A2,B2,Y2,X2;

    Subsystems subsytems;
    Dashboard dashboard;

    @Override
    public void init() {
        //chassis.resetEncoders();
        //subsytems = new Subsystems(hardwareMap);
        chassis = new ChassisController(hardwareMap);
        chassis.resetEncoders();

    }

    @Override
    public void loop() {
        Dashboard.initDashboard();
        Dashboard.packet.put("Current positionX",chassis.getDistanceInchesX());
        Dashboard.packet.put("Current positionY", chassis.getDistanceInchesY());
        //Dashboard.moveCircle(chassis.getDistanceInchesX(),chassis.getDistanceInchesY());



    }
    public void waitFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.seconds() < seconds) {
            System.out.println("Hewo :)");
        }
    }

}

