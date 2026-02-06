package org.firstinspires.ftc.teamcode.RobotMode.TELEOP;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.bearing;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.range;
import static org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter.timer;
import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.dashboardTelemetry;
import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.ftcDashboard;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;

import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;

@TeleOp(name="Blue TeleOp",group="Blue")
public class Blue_Alliance_TeleOp extends OpMode {

    ChassisController chassis;
    Camera_Detection cameraDetection;
    TelemetryMethods telemetryMethods;
    Subsystems subsystems;

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
    boolean isSlowActive;
    public static boolean follow;
    boolean autoMode = false;

    Dashboard dashboard;

    ElapsedTime timer0 = new ElapsedTime();

    @Override
    public void init() {
        chassis = new ChassisController(hardwareMap);
        cameraDetection = new Camera_Detection(hardwareMap);

        subsystems = new Subsystems(hardwareMap);
        telemetryMethods = new TelemetryMethods();
        telemetry = new MultipleTelemetry(telemetry,dashboardTelemetry);



        telemetryMethods.InitTelemetry(telemetry);

        timer0.reset();
    }

    @Override
    public void loop() {
        updateControllerInput();
        Dashboard.initDashboard(1, 1,10,10);

        telemetryMethods.TelemetryShooter(telemetry);
        telemetryMethods.TelemetryUpdateCamera();

        cameraDetection.CameraDetectionBlue();
        ftcDashboard.sendImage(cameraDetection.streamProcessor.getLastFrame());

        //// Chassis

        if (RB1) {
            isSlowActive = true;
        } else if (LB1) {
            isSlowActive = false;
        } else if (gamepad1.xWasPressed()){
            follow = !follow;
        }

        if(follow){
            chassis.mecanumFollow(bearing);
        } else if(isSlowActive){
            chassis.slowMode(LSx1, LSy1, RSx1);
        } else {
            chassis.mecanumDrive(LSx1, LSy1, RSx1);
        }

        //// Subsystems

        if (!autoMode){
            if (LSy2 != 0) {
                subsystems.intake.moveIntake(Math.abs(LSy2) > .2 ? LSy2 : 0);
                subsystems.indexer.moveIndexer(Math.abs(LSy2) > .2 ? LSy2 : 0);
            }   else if (RSy2 != 0){
                subsystems.feeder.moveFeeder(Math.abs(LSy2) > .2 ? LSy2 : 0);
            } else {
                subsystems.stopCycling();
            }
        }

        if (!autoMode){
            if (RSy2 != 0) {
                subsystems.feeder.moveFeeder(Math.abs(RSy2) > .2 ? RSy2 : 0);
            } else {
                subsystems.feeder.stopFeeder();
            }
        }

        if (RT2 != 0) {
            autoMode = true;
            subsystems.shooter.shoot(range);
            subsystems.intake.moveIntake(Shooter.getActualVel() >= (Shooter.controlPoints.get(range) - 700) ? 1 : 0);
            subsystems.indexer.moveIndexer(Shooter.getActualVel() >= (Shooter.controlPoints.get(range) - 700) ? 1 : 0);
            subsystems.feeder.moveFeeder(Shooter.getActualVel() >= (Shooter.controlPoints.get(range) - 700) ? 1 : 0);
        } else {
            autoMode = false;
            subsystems.shooter.stopShooter();
        }

        if (LT2 != 0) {
            subsystems.shooter.configShooter();
        } else {
            subsystems.shooter.stopShooter();
        }

    }
    public void updateControllerInput(){

        RT1 = gamepad1.right_trigger;
        LT1 = gamepad1.left_trigger;
        LSx1 = gamepad1.left_stick_x;
        LSy1 = -gamepad1.left_stick_y;
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

