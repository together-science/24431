package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.decode.BaseAutoDecode;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Arrays;

@Autonomous(name = "AutoAim", group = "robot")
public class AutoAim extends BaseAutoDecode {
    @Override
    protected void auto(){
        // 1000 -102.5
        sleep(5000);
        chassis.autoAim(Arrays.asList(20, 24));

    }
}