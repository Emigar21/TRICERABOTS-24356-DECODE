package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotFunctions.ChassisController;

@TeleOp(name="Main",group="Robot")
public class Main extends OpMode {

    ChassisController chassis;
    Telemetry dashboardTelemetry;

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

    @Override
    public void init() {
        chassis.resetEncoders();
        subsytems = new Subsystems(hardwareMap);
//        chassis = new ChassisController(hardwareMap);
    }

    @Override
    public void loop() {
        LSx1 = gamepad1.left_stick_x;
        LSy1 = gamepad1.left_stick_y;
        RSx1 = gamepad1.right_stick_x;
        chassis.mecanumDrive(-LSx1,LSy1,-RSx1);
        if (RT1 < 0 || LT1 < 0 ){
            subsytems.moveIntake(RT1, -LT1);
        }


    }

}

