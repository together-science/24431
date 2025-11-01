package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseAuto extends LinearOpMode {
    protected ScorpChassis chassis = null;

    // this method will be implemented by the subclass
    protected void autoInit() {
        // not abstract because not required in subclass
        chassis = new ScorpChassis(this, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        chassis.init();
    }

    // this method will be implemented by the subclass
    protected abstract void auto();

    @Override
    public void runOpMode() {
        autoInit();

        telemetry.addData(">", "Waiting for start ...");
        telemetry.update();

        while (opModeInInit()) {
            sleep(50);
        }

        auto();
    }
}
