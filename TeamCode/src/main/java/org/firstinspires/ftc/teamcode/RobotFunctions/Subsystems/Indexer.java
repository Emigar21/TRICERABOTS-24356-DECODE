package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class Indexer {
    public static CRServo indexerServo;
    public Indexer (HardwareMap hardwareMap){
        indexerServo = hardwareMap.get(CRServo.class,"indexerServo");

        indexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveIndexer (double power){ indexerServo.setPower(power); }

    public void stopIndexer(){ indexerServo.setPower(0); }
}
