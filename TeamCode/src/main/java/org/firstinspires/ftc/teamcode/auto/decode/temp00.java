package org.firstinspires.ftc.teamcode.auto.decode;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOtos;

public class temp00 extends BaseAutoDecode {
    double turn, turn2, xStart, xEnd, yStart, yEnd;
    long sleep;

    @Override
    protected void auto(){
        devices.leftCannon.spinUp();
        devices.rightCannon.spinUp();
        chassis.strafeTo(xStart, yStart, ScorpChassisOtos.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, turn);
        devices.leftCannon.fire();
        sleep(sleep);
        devices.rightCannon.fire();
        sleep(sleep);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, turn2);
        chassis.strafeTo(0, 0, ScorpChassisOtos.DRIVE_SPEED_FAST);
    }
    void run(boolean blue){
        xStart = 0;
        yStart = 72;
        xEnd = 0;
        yEnd = 12;
        turn = blue ? 45 : 315;
        turn2 = blue ? -90 : 90;
        sleep = 1000;
    }
}