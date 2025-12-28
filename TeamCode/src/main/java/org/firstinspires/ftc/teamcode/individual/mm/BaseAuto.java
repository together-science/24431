package org.firstinspires.ftc.teamcode.individual.mm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseAuto extends LinearOpMode {
    protected void autoInit(){}
    protected abstract void auto();
    @Override
    public void runOpMode(){
        autoInit();
        while(opModeInInit()) sleep(50);
        auto();
    }
}