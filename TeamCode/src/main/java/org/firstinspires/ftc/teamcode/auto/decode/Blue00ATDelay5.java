package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "_Blue00Delay5", group="Robot")
public class Blue00ATDelay5 extends Blue00AT {
    @Override
    protected void auto() {
        sleep(5000);
        super.auto();
    }
}