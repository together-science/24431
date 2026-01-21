package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "_Red02Delay5", group="Robot")
public class Red02ATDelay5 extends Red02AT {
    @Override
    protected void auto(){
        sleep(5000);
        super.auto();
    }
}