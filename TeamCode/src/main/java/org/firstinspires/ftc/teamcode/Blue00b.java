package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue00b", group="Robot")
public class Blue00b extends Blue00{
    @Override
    protected void auto(){
        sleep(10000);
        super.auto();
    }
}
