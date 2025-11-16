package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class temp00 extends BaseAuto25 {
    double turn, xStart, xEnd, yStart, yEnd;
    long sleep;

    @Override
    protected void auto(){
        chassis.strafeTo(xStart, yStart, ScorpChassis.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, turn);
        rightCannon.fire();
        sleep(sleep);
        leftCannon.fire();
        chassis.strafeTo(xEnd, yEnd, ScorpChassis.DRIVE_SPEED_NORMAL);
    }
    void run(boolean blue){
        xStart = blue ? 12 : -12;
        yStart = 48;
        xEnd = 0;
        yEnd = 12;
        turn = blue ? 45 : 315;
        sleep = 500;
    }
}