package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue00a", group="Robot")
public class Blue00a extends Blue00{
    @Override
    protected void auto(){
        sleep(2000);
        super.auto();
    }
}
