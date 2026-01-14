package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOtos;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpCannon;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red01AT", group="Robot")
public class Red01AT extends BaseAutoDecode {
    @Override
    protected void auto(){
        chassis.strafeTo(0, -45, ScorpChassisOtos.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, 45);
        getObeliskId();
        devices.leftCannon.spinUp();
        devices.rightCannon.spinUp();
        getObeliskId();
        chassis.turnToHeading(0.5, 0);
        fire();
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, 45);
        chassis.strafeTo(11, -65, ScorpChassisOtos.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, 145);
    }
}// Pick the square of a number between 1 and 9
 // The Ghost of Ronald McDonald 1997 will haunt you in (root(choice)/2) days