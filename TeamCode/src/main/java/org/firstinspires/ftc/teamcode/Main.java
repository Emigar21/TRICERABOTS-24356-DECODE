package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.bearing;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.RobotFunctions.ChassisController;

import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;
import org.firstinspires.ftc.teamcode.Variables.ConfigVariables;

@TeleOp(name="Main",group="Robot")
public class Main extends OpMode {

    ChassisController chassis;


    // Controller Input

    // Gamepad 1
    double LT1, RT1;
    double LSx1, LSy1, RSx1, RSy1;
    boolean LB1, RB1;
    public static boolean A1;
    public static boolean B1;
    static boolean Y1;
    static boolean X1;
    boolean dPadUp1, dPadDown1, dPadRight1, dPadLeft1;

    // Gamepad 2
    float LT2, RT2;
    double LSx2, LSy2, RSx2, RSy2;
    boolean LB2, RB2;
    boolean dPadUp2, dPadDown2, dPadRight2, dPadLeft2;
    boolean A2,B2,Y2,X2;

    Subsystems subsytems;
    Dashboard dashboard;
    boolean isAlive = false;
    boolean timed0 = false;
    boolean initTimer0 = false;

    ElapsedTime timer0 = new ElapsedTime();

    @Override
    public void init() {
        chassis = new ChassisController(hardwareMap);
        subsytems = new Subsystems(hardwareMap);
        //chassis.resetEncoders();
        timer0.reset();

    }

    @Override
    public void loop() {
        updateControllerInput();
       // Dashboard.initDashboard(chassis.getDistanceInchesX(), chassis.getDistanceInchesY(),10,10);

//        if (A1) {
//            isAlive = true;
//        }
//        if (isAlive){
//            ChassisController.mecanumDrive(LSx1, -LSy1, bearing);
//        }  else if (Math.abs(bearing) <= 0.2) {
//            isAlive = false;
//        } else if (B1) {
//            isAlive = false;
//        } else {
//            ChassisController.mecanumDrive(LSx1, -LSy1, RSx1);
//        }
        //TODO: probar la programacion en chassis con camara y verificar que funcione


        subsytems.moveIntake(RB2 ? 1:0);
        subsytems.moveIndexer(LB2 ? 1:0);
        subsytems.moveFeeder(Y2 ? 1:0);


        if ((B1||A1) && !initTimer0) {
            initTimer0 = true;
            timer0.reset();
        }

        if (B1) {
            subsytems.moveShooterLong(); //0.87
            if (timer0.seconds() > ConfigVariables.timeerShooter) {  //1.85s
                subsytems.moveFeeder(1);
            }
        } else if (A1){
            subsytems.moveShooterLong(); //0.87
            if (timer0.seconds() > ConfigVariables.timeerShooter) {  //1.85s
                subsytems.moveFeeder(1);
            }

        } else{
            subsytems.moveFeeder (0);
            subsytems.shooterMotor.setPower(0);
            initTimer0 = false;
        }

        //Dashboard.packet.put("color", chassis.getColor());
    }
    public void waitFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.seconds() < seconds) {
            System.out.println("Hewo :)");
        }
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

