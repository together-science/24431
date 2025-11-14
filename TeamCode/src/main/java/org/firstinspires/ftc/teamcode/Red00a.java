package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red00a", group = "robot")
public class Red00a extends Red00 {
    @Override
    protected void auto(){
        sleep(2000);
        super.auto();
    }
}
