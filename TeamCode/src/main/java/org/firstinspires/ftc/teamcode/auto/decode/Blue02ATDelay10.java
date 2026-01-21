package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "_Blue02Delay10", group="Robot")
public class Blue02ATDelay10 extends Blue02AT {
    @Override
    protected void auto() {
        sleep(10000);
        super.auto();
    }
}