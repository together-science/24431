package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue01b", group = "robot")
public class Blue01b extends Blue01 {
    @Override
    protected void auto(){
        sleep(10000);
        super.auto();
    }
}
