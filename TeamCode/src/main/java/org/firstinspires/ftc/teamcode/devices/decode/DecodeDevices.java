package org.firstinspires.ftc.teamcode.devices.decode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DecodeDevices {
    public ScorpCannonMT leftCannon = null;
    public ScorpCannonMT rightCannon = null;
    public ScorpMotorIntake intake = null;
    public ScorpSorter sorter = null;
    public ScorpCamera camera = null;

    public DecodeDevices(LinearOpMode op) {
        leftCannon = new ScorpCannonMotorPort(op, "left_cannon_wheel",
                "left_cannon_trigger", 0.90, DcMotorSimple.Direction.FORWARD);
        rightCannon = new ScorpCannonMotorPort(op, "right_cannon_wheel",
                "right_cannon_trigger", 0.88, DcMotorSimple.Direction.REVERSE);
        intake = new ScorpMotorIntake(op, "left_intake", "right_intake");
        sorter = new ScorpSorter(op, "sorter_servo");
        camera = new ScorpCamera(op, "Webcam 1");
    }
}