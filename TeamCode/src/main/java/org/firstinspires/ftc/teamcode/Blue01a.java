package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue01a", group = "robot")
public class Blue01a extends Blue01 {
    @Override
    protected void auto(){
        sleep(2000);
        super.auto();
    }
}
