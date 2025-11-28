package org.firstinspires.ftc.teamcode.auto.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "A00 Preload", group="Robot")
public class A00Preload extends BaseAutoDecode {
    @Override
    protected void auto(){
        devices.leftCannon.cannonIntake(true);
        devices.rightCannon.cannonIntake(true);
        sleep(10000);
        devices.leftCannon.spinDown();
        devices.rightCannon.spinDown();
    }
}