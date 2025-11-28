package org.firstinspires.ftc.teamcode.auto.decode;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOtos;

// NOTICE
// DON'T USE THIS IN A GAME UNTIL IT'S
// VARIABLES HAVE THE CORRECT VALUES

public class temp01 extends BaseAutoDecode {
    double turn, xStart, yStart, xEnd, yEnd;
    long delay;

    @Override
    protected void auto(){
        chassis.strafeTo(xStart, yStart, ScorpChassisOtos.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, turn);
        devices.leftCannon.fire();
        sleep(delay);
        devices.rightCannon.fire();
        chassis.strafeTo(xEnd, yEnd, ScorpChassisOtos.DRIVE_SPEED_NORMAL);
    }
    void run(boolean blue){
        xStart = blue ? 0 : -1;
        yStart = blue ? 0 : -1;
        xEnd = blue ? 0 : -1;
        yEnd = blue ? 0 : -1;
        turn = blue ? 0 : -1;
        delay = blue ? 0 : -1;
    }
}
