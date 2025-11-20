package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Mr. Mein, don't get mad, I only work on this when I have free time

@Autonomous(name = "mmTestAuto", group = "robot")
public class mmTestAuto extends mmBaseAuto {
    @Override
    protected void auto(){
        chassis.testDrive();
    }
}