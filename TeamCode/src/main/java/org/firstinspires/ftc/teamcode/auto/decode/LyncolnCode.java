package org.firstinspires.ftc.teamcode.auto.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "LyncolnCode", group = "test")
public class LyncolnCode extends BaseAutoDecode {
    @Override
    protected void auto() {
       //chassis.strafeTo(40, 40, ScorpChassisBase.DRIVE_SPEED_NORMAL, 0);
       //chassis.strafeTo(45, 45, ScorpChassisBase.DRIVE_SPEED_NORMAL, 45);
       //chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_SLOW, 45);
       //chassis.strafeTo(70, 45, ScorpChassisBase.DRIVE_SPEED_NORMAL, 0);
       //sleep(2000);
       //fire();
       //chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_FAST, 0);
       //chassis.strafeTo(10, 10, ScorpChassisBase.DRIVE_SPEED_FAST, 0);
       //chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_FAST, -100);
       //chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_SLOW, -50);
       //chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_NORMAL, 0);
       //chassis.strafeTo(0, 0, ScorpChassisBase.DRIVE_SPEED_NORMAL, 0);
        chassis.strafeTo(10, 0, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(10, 10, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(0, 10, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(0, 0, ScorpChassisBase.DRIVE_SPEED_FAST);
        sleep(2000);
        chassis.strafeTo(10, 0, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(20, 0, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(20, 10, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(20, 20, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(10, 20, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(0, 20, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(0, 10, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(0, 0, ScorpChassisBase.DRIVE_SPEED_FAST);
        sleep(2000);
        chassis.strafeTo(10, 0, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(20, 0, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(30, 0, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(30, 10, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(30, 20, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(30, 30, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(20, 30, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(10, 30, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(0, 30, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(0, 20, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(0, 10, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.strafeTo(0, 0, ScorpChassisBase.DRIVE_SPEED_FAST);
    }
}
