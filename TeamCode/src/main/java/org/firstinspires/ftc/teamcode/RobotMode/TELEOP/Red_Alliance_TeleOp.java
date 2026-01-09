package org.firstinspires.ftc.teamcode.RobotMode.TELEOP;

import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.dashboardTelemetry;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.SubsystemInitializer;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;

import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;

@TeleOp(name="RedTeleOp",group="TeleOps")
public class Red_Alliance_TeleOp extends OpMode {

    ChassisController chassis;
    Camera_Detection cameraDetection;
    TelemetryMethods telemetryMethods;

    SubsystemInitializer subsystemInitializer;

    // Controller Input

    // Gamepad 1
    double LT1, RT1;
    double LSx1, LSy1, RSx1, RSy1;
    boolean LB1, RB1;
    boolean A1, B1, Y1, X1;
    boolean dPadUp1, dPadDown1, dPadRight1, dPadLeft1;

    // Gamepad 2
    float LT2, RT2;
    double LSx2, LSy2, RSx2, RSy2;
    boolean LB2, RB2;
    boolean dPadUp2, dPadDown2, dPadRight2, dPadLeft2;
    boolean A2,B2,Y2,X2;


    Dashboard dashboard;

    ElapsedTime timer0 = new ElapsedTime();

    @Override
    public void init() {
        chassis = new ChassisController(hardwareMap);
        cameraDetection = new Camera_Detection(hardwareMap);

        subsystemInitializer = new SubsystemInitializer(hardwareMap);
        telemetryMethods = new TelemetryMethods();
        Sensors sensors = new Sensors(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry,dashboardTelemetry);

        timer0.reset();
    }

    @Override
    public void loop() {

        updateControllerInput();

    }
    public void updateControllerInput(){
        RT1 = gamepad1.right_trigger;
        LT1 = gamepad1.left_trigger;
        LSx1 = gamepad1.left_stick_x;
        LSy1 = gamepad1.left_stick_y;
        RSx1 = gamepad1.right_stick_x;
        RSy1 = gamepad1.right_stick_y;
        LB1 = gamepad1.left_bumper;
        RB1 = gamepad1.right_bumper;
        A1 = gamepad1.a;
        B1 = gamepad1.b;
        X1 = gamepad1.x;
        Y1 = gamepad1.y;
        dPadUp1 = gamepad1.dpad_up;
        dPadRight1 = gamepad1.dpad_right;
        dPadDown1 = gamepad1.dpad_down;
        dPadLeft1 = gamepad1.dpad_left ;

        RT2 = gamepad2.right_trigger;
        LT2 = gamepad2.left_trigger;
        LSx2 = gamepad2.left_stick_x;
        LSy2 = -gamepad2.left_stick_y;
        RSy2 = -gamepad2.right_stick_y;
        LB2 = gamepad2.left_bumper;
        RB2 = gamepad2.right_bumper;
        RSx2 = gamepad2.right_stick_x;
        dPadUp2 = gamepad2.dpad_up;
        dPadDown2 = gamepad2.dpad_down;
        dPadLeft2 = gamepad2.dpad_left;
        dPadRight2 = gamepad2.dpad_right;
        A2 = gamepad2.a;
        B2 = gamepad2.b;
        X2 = gamepad2.x;
        Y2 = gamepad2.y;
    }
}

