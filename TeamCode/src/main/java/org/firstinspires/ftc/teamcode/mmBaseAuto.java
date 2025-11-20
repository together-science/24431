package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Mr. Mein, don't get mad, I only work on this when I have free time

public abstract class mmBaseAuto extends LinearOpMode {
    protected mmScorpChassis chassis = null;
    protected ScorpCannonMT leftCannon = null;
    protected ScorpCannonMT rightCannon = null;
    protected ScorpIntake intake = null;
    protected ScorpSorter sorter = null;
    protected ScorpCamera camera = null;

    protected void autoInit(){
        chassis = new mmScorpChassis(this, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        leftCannon = new ScorpCannonMotorPort(this, "left_cannon_wheel", "left_cannon_trigger", 1.0, DcMotorSimple.Direction.FORWARD);
        rightCannon = new ScorpCannonMotorPort(this, "right_cannon_wheel", "right_cannon_trigger", 0.75, DcMotorSimple.Direction.REVERSE);
        intake = new ScorpMotorIntake(this, "left_intake", "right_intake");
        sorter = new ScorpSorter(this, "sorter_servo");
        camera = new ScorpCamera(this, "camera");
        chassis.init();
    }
    protected abstract void auto();
    @Override
    public void runOpMode(){
        autoInit();
        while(opModeInInit()) sleep(50);
        auto();
    }
}
