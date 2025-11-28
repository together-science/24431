package org.firstinspires.ftc.teamcode.individual.mm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class mmBaseAuto extends LinearOpMode {
    protected mmScorpChassis chassis = null;
    protected mmScorpCannon leftCannon = null;

    protected void autoInit(){
        chassis = new mmScorpChassis(this, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        leftCannon = new mmScorpCannon();
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
