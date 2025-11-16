package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AA00 TestAuto", group = "robot")
public class A00TestAuto extends BaseAuto25 {
    @Override
    protected void auto() {
        chassis.strafeTo(0, 24, ScorpChassis.DRIVE_SPEED_NORMAL);

    }
}