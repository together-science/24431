package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "_Red02Delay10", group="Robot")
public class Red02ATDelay10 extends Red02AT {
    @Override
    protected void auto(){
        sleep(10000);
        super.auto();
    }
}