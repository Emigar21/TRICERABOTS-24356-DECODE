package org.firstinspires.ftc.teamcode.RobotMode.TELEOP;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.bearing;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.range;
import static org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter.timer;
import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.dashboardTelemetry;
import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.ftcDashboard;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;

import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;

@TeleOp(name="RedTeleOp",group="TeleOps")
public class Red_Alliance_TeleOp extends OpMode {

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
        telemetryMethods.TelemetryCyclying(telemetry);
        telemetryMethods.TelemetryUpdateCamera(telemetry);

        cameraDetection.CameraDetectionRed();
        ftcDashboard.sendImage(cameraDetection.streamProcessor.getLastFrame());

        if(B1){
            chassis.stopMotors();
        }  else if(X1) {
            timer.reset();
            follow = !follow;
            while (timer.seconds()< .1){
                chassis.chassisFollow(bearing);
            }
        } else if(follow){
            chassis.chassisFollow(bearing);
        } else if (Math.abs(LSx1) > .2 || Math.abs(LSy1) > .2 || Math.abs(RSx1) > .2 ){
            chassis.mecanumDrive(
                    LB1 ? LSx1 * .3  : LSx1  ,
                    LB1 ? LSy1 * .3 : LSy1 ,
                    LB1 ? RSx1 * .3 : RSx1
            );
        } else {

            chassis.stopMotors();
        }

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

        if (A2 && Math.abs(LSy2) > .2) {
            subsystems.indexer.moveIndexer(LSy2);
            subsystems.intake.stopIntake();
        } else if (X2 && Math.abs(LSy2) > .2) {
            subsystems.indexer.stopIndexer();
            subsystems.intake.moveIntake(LSy2);
        }   else if (Math.abs(LSy2) > .2){
            subsystems.intake.moveIntake(LSy2);
            subsystems.indexer.moveIndexer(LSy2);
        } else {
            subsystems.intake.stopIntake();
            subsystems.indexer.stopIndexer();
        }

        if (Math.abs(RSy2) > .1) {
            subsystems.feeder.moveFeeder(-RSy2);
        } else {
            subsystems.feeder.stopFeeder();
        }

        if (RT2 != 0) {
            subsystems.shooter.shoot(range);
        } else {
            subsystems.shooter.stopShooter();
        }

    }
    public void updateControllerInput(){
        RT1 = gamepad1.right_trigger;
        LT1 = gamepad1.left_trigger;
        LSx1 = -gamepad1.left_stick_x;
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
        RSy2 = gamepad2.right_stick_y;
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

