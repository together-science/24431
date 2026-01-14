package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOtos;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpCannon;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red00AT", group="Robot")
public class Red00AT extends BaseAutoDecode {
    @Override
    protected void auto(){
        getObeliskId();
        devices.leftCannon.spinUp();
        devices.rightCannon.spinUp();
        chassis.strafeTo(0, 62, ScorpChassisOtos.DRIVE_SPEED_FAST);
        getObeliskId();
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, -45);
        fire();
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, 0);
        chassis.strafeTo(0, 15, ScorpChassisOtos.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, 100);
    }
}