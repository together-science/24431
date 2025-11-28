package org.firstinspires.ftc.teamcode.individual.mm;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "mmTestAuto", group = "robot")
public class mmTestAuto extends mmBaseAuto {
    @Override
    protected void auto(){
        chassis.testDrive();
        sleep(2000);
    }
}