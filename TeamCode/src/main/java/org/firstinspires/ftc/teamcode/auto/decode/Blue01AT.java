package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;
import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOtos;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpCannon;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Blue01AT", group="Robot")
public class Blue01AT extends BaseAutoDecode {
    @Override
    protected void auto(){
        chassis.strafeTo(0, -45, ScorpChassisOtos.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, -45);
        getObeliskId();
        devices.leftCannon.spinUp();
        devices.rightCannon.spinUp();
        getObeliskId();
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, 0);
        fire();
        //chassis.strafeTo(-24, 0, ScorpChassisOtos.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, -145);
//        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, -45);
//        chassis.strafeTo(-11, -65, ScorpChassisOtos.DRIVE_SPEED_FAST);
//        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, -145);
    }
}