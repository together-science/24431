package org.firstinspires.ftc.teamcode;

public abstract class BaseAuto25 extends BaseAuto {
    protected ScorpCannon leftCannon = null;
    protected ScorpCannon rightCannon = null;
    protected ScorpIntake intake = null;
    protected ScorpSorter sorter = null;
    protected ScorpCamera camera = null;
    //protected ScorpColor color = null;

    // this method will be implemented by the subclass
    protected void autoInit() {
        super.autoInit();
        leftCannon = new ScorpCannon(this, "left_cannon_wheel", "left_cannon_trigger");
        rightCannon = new ScorpCannon(this, "right_cannon_wheel", "right_cannon_trigger");
        intake = new ScorpIntake(this, "left_intake", "right_intake");
        sorter = new ScorpSorter(this, "sorter_servo");
        camera = new ScorpCamera(this, "camera");
        // color = new ScorpColorTestMM(hardwareMap, "sensor_color_not_real");
        // I commented out the init ^^^ because sensor_color_not_real, is not real so when it tires to find it it will run an error.
    }
}
