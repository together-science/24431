package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class BaseAuto25 extends BaseAuto {

    protected ScorpCannonMT leftCannon = null;
    protected ScorpCannonMT rightCannon = null;
    protected ScorpIntake intake = null;
    protected ScorpSorter sorter = null;
    protected ScorpCamera camera = null;

    @Override
    protected void autoInit() {
        super.autoInit();
        leftCannon = new ScorpCannonMotorPort(this, "left_cannon_wheel",
                "left_cannon_trigger", 0.90, DcMotorSimple.Direction.FORWARD);
        rightCannon = new ScorpCannonMotorPort(this, "right_cannon_wheel",
                "right_cannon_trigger", 0.88, DcMotorSimple.Direction.REVERSE);
        intake = new ScorpMotorIntake(this, "left_intake", "right_intake");
        sorter = new ScorpSorter(this, "sorter_servo");
        camera = new ScorpCamera(this, "Webcam 1");
    }

    @Override
    protected void autoStart() {
        super.autoStart();
        leftCannon.reset();
        rightCannon.reset();
    }
}
