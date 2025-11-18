package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseAuto extends LinearOpMode {
    protected ScorpChassis chassis = null;
    protected void autoInit(){
        chassis = new ScorpChassis(this, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        chassis.init();
    }
    protected abstract void auto();

    @Override
    public void runOpMode() {
        autoInit();

        telemetry.addData(">", "Waiting for start ...");
        telemetry.update();

        while (opModeInInit()) sleep(50);

        auto();
    }
}
