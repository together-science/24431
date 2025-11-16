package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// NOTICE
// DON'T USE THIS IN A GAME UNTIL IT'S
// VARIABLES HAVE THE CORRECT VALUES

public class temp01 extends BaseAuto25 {
    double turn, xStart, yStart, xEnd, yEnd;
    long delay;

    @Override
    protected void auto(){
        chassis.strafeTo(xStart, yStart, ScorpChassis.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, turn);
        leftCannon.fire();
        sleep(delay);
        rightCannon.fire();
        chassis.strafeTo(xEnd, yEnd, ScorpChassis.DRIVE_SPEED_NORMAL);
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
