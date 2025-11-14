package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "A00 Preload", group="Robot")
public class A00Preload extends BaseAuto25 {
    @Override
    protected void auto(){
        leftCannon.cannonIntake(true);
        rightCannon.cannonIntake(true);
        sleep(10000);
        leftCannon.spinDown();
        rightCannon.spinDown();
    }
}