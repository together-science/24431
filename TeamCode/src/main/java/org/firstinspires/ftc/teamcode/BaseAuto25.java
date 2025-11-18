package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class BaseAuto25 extends BaseAuto {

    protected ScorpCannon leftCannon = null;
    protected ScorpCannon rightCannon = null;
    protected ScorpIntake intake = null;
    protected ScorpSorter sorter = null;
    protected ScorpCamera camera = null;
    protected void autoInit() {
        super.autoInit();
        leftCannon = new ScorpCannonMotorPort(this, "left_cannon_wheel", "left_cannon_trigger", 1.0, DcMotorSimple.Direction.FORWARD);
        rightCannon = new ScorpCannonMotorPort(this, "right_cannon_wheel", "right_cannon_trigger", 0.75, DcMotorSimple.Direction.REVERSE);
        intake = new ScorpMotorIntake(this, "left_intake", "right_intake");
        sorter = new ScorpSorter(this, "sorter_servo");
        camera = new ScorpCamera(this, "camera");
    }
}
