package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "_Red00Delay5", group="Robot")
public class Red00ATDelay5 extends Red00AT {
    @Override
    protected void auto(){
        sleep(5000);
        super.auto();
    }
}