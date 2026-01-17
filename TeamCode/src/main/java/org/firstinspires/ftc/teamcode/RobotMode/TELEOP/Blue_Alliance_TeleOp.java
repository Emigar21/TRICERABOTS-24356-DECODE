package org.firstinspires.ftc.teamcode.RobotMode.TELEOP;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.bearing;
import static org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator.compensateVoltage;
import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.dashboardTelemetry;
import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.ftcDashboard;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.power;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;

import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;

@TeleOp(name="BlueTeleOp",group="TeleOps")
public class Blue_Alliance_TeleOp extends OpMode {

    ChassisController chassis;
    Subsystems subsystems;
    Camera_Detection cameraDetection;

    TelemetryMethods telemetryMethods;
    boolean isSlowActive = false;

    // Controller Input

    // Gamepad 1
    double LT1, RT1;
    public double LSx1;
    public static double LSy1;
    public static double RSx1;
    public double RSy1;
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

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        chassis = new ChassisController(hardwareMap);
        cameraDetection = new Camera_Detection(hardwareMap);
        subsystems = new Subsystems(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry,dashboardTelemetry);
        telemetryMethods = new TelemetryMethods();

        telemetryMethods.InitTelemetry(telemetry);

        timer.reset();
    }

    @Override
    public void loop() {

        updateControllerInput();
        Dashboard.initDashboard(chassis.getDistanceInchesX(), chassis.getDistanceInchesY(),10,10);

        telemetryMethods.ClearTelemetry(telemetry);
        telemetryMethods.TelemetryShooter(telemetry);
        telemetryMethods.TelemetryCyclying(telemetry);

        cameraDetection.CameraDetectionBlue();
        ftcDashboard.sendImage(cameraDetection.streamProcessor.getLastFrame());


//        if(B1){
//            chassis.stopMotors();
//        }  else if(X1) {
//            follow = !follow;
//        } else if(follow){
//            chassis.chassisFollow(bearing);
//        } else if (Math.abs(LSx1) > .2 || Math.abs(LSy1) > .2 || Math.abs(RSx1) > .2 ){
//            chassis.mecanumDrive(
//                    LB1 ? LSx1 * .3 * (timer.seconds() * 2) : LSx1 * (timer.seconds() * 2),
//                    LB1 ? LSy1 * .3 * (timer.seconds() * 2): LSy1 * (timer.seconds() * 2),
//                    LB1 ? RSx1 * .3 * (timer.seconds() * 2): RSx1 * (timer.seconds() * 2)
//            );
//        } else {
//            timer.reset();
//            chassis.stopMotors();
//        }

        ////chassis

        if (RB1) {
            isSlowActive = true;
        } else if (LB1) {
            isSlowActive = false;
        }
        if (isSlowActive)
            chassis.slowMode(LSx1, LSy1, RSx1);
        else {
            chassis.mecanumDrive(LSx1*2, LSy1*2, RSx1*2);
        }
        ////subsystems

        if (A2 && LSy2 != 0) {
            subsystems.indexer.moveIndexer(LSy2);
            subsystems.intake.stopIntake();
        } else if (X2 && LSy2 != 0) {
            subsystems.indexer.stopIndexer();
            subsystems.intake.moveIntake(LSy2);
        }   else if (LSy2 != 0){
            subsystems.intake.moveIntake(LSy2);
            subsystems.indexer.moveIndexer(LSy2);
        } else {
            subsystems.intake.stopIntake();
            subsystems.indexer.stopIndexer();
        }

        if (RSy2 != 0) {
            subsystems.feeder.moveFeeder(RSy2);
        } else {
            subsystems.feeder.stopFeeder();
        }

        if (RT2 != 0) {
            subsystems.shooter.shoot();
        } else {
            subsystems.shooter.stopShooter();
        }

//        if(B2){
//            subsystems.stopCycling();
//        } else if (Y2){
//            subsystems.intake.moveIntake(LSy2);
//        } else if (X2) {
//            subsystems.indexer.moveIndexer(LSy2);
//        } else if (RSy2 != 0 ){
//          subsystems.feeder.moveFeeder(RSy2);
//        } else if (LSy2 != 0){
//            subsystems.intake.moveIntake(LSy2);
//            subsystems.indexer.moveIndexer(LSy2);
//        } else {
//            subsystems.stopCycling();
//        }
//
//        if(RT2 > .2) {
//        subsystems.shooter.shoot(RT2);
////        } else if (RT2 > .2){
////            subsystems.shooter.shoot(compensateVoltage(power));
////        }
//            } else {
//            subsystems.shooter.stopShooter();
//        }


    }

    public void updateControllerInput(){
        RT1 = gamepad1.right_trigger;
        LT1 = gamepad1.left_trigger;
        LSx1 = -+gamepad1.left_stick_x;
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

