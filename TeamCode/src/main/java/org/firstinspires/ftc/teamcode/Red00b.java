package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red00b", group = "robot")
public class Red00b extends Red00 {
    @Override
    protected void auto(){
        sleep(10000);
        super.auto();
    }
}
