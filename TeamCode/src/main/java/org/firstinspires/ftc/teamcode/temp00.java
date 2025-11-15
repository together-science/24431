package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class temp00 extends BaseAuto25 {
    double turn, xStart, xEnd, yStart, yEnd;
    long sleep;

    @Override
    protected void auto(){
        chassis.strafeTo(xStart, yStart, ScorpChassis.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, turn); // right 45
        rightCannon.fire();
        sleep(sleep);
        leftCannon.fire();
        chassis.moveTo(xEnd, yEnd, ScorpChassis.DRIVE_SPEED_NORMAL);
    }
    void run(boolean blue){
        if(blue){
            xStart = 12; yStart = 48; xEnd = -12; yEnd = 12;
            turn = 45; sleep = 2000;
        }
        else{
            xStart = -12; yStart = 48; xEnd = 12; yEnd = 12;
            turn = 315; sleep = 2000;
        }
        auto();
    }
}
