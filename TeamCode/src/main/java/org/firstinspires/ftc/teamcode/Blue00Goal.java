package org.firstinspires.ftc.teamcode;

/*
This is Matthew's auto, dont change it.
I know it hardly works, I am working on it possibly.
Making WORKING code was not an expected requirement.
How do I have so much to do and at the same time have nothing to do?
hoursSpent = 1;
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "A00 Blue Goal", group="Robot")
public class Blue00Goal extends BaseAuto25 {
    @Override
    protected void auto(){
        chassis.moveTo(0, 12, ScorpChassis.DRIVE_SPEED_NORMAL);
        sleep(10000);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 45); // Left 45
        // chassis.
    }
}