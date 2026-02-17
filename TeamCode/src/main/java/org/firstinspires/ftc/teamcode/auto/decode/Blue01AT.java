package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

import java.util.Arrays;

@Autonomous(name = "_Blue01", group="Robot")
public class Blue01AT extends BaseAutoDecode {
    @Override
    protected void auto(){
        // get cannons running
        devices.leftCannon.spinUp();
        devices.rightCannon.spinUp();

        // drive forward to firing point
        chassis.strafeTo(-45, 0, ScorpChassisBase.DRIVE_SPEED_FAST);

        // look at obelisk and try to get id
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_NORMAL, -45);
        getObeliskId();

        // turn back to goal, fire
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_NORMAL, 45);
        chassis.autoAim(Arrays.asList(20, 24));
        fire();

        // turn back towards obelisk, drive backward close to base, turn toward human player
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_NORMAL, -45);
        chassis.strafeTo(0, -53, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_NORMAL, -100);
    }
}