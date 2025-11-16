package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red00", group = "robot")
public class Red00 extends temp00 {
    @Override
    protected void auto(){
        super.run(false);
        super.auto();
    }
}
