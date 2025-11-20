package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class temp00 extends BaseAuto25 {
    double turn, turn2, xStart, xEnd, yStart, yEnd;
    long sleep;

    @Override
    protected void auto(){
        leftCannon.spinUp();
        rightCannon.spinUp();
        chassis.strafeTo(xStart, yStart, ScorpChassis.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, turn);
        leftCannon.fire();
        sleep(sleep);
        rightCannon.fire();
        sleep(sleep);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, turn2);
        chassis.strafeTo(0, 0, ScorpChassis.DRIVE_SPEED_FAST);
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