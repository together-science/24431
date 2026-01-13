package org.firstinspires.ftc.teamcode.individual.mm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseAuto extends LinearOpMode {
    Chassis chassis = null;

    BaseAuto(){
        chassis = new Chassis(this,
                "left_front_drive",
                "right_front_drive",
                "left_back_drive",
                "right_back_drive",
                "imu");
    }
    protected void autoInit(){}
    protected abstract void auto();
    @Override
    public void runOpMode(){
        autoInit();
        while(opModeInInit()) sleep(50);
        auto();
    }
}