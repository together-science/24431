package org.firstinspires.ftc.teamcode;

/*
This is Matthew's auto, dont change it.
I know it hardly works, I am working on it possibly.
Making WORKING code was not an expected requirement.
How do I have so much to do and at the same time have nothing to do?
hoursSpent = 1;
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "A00 Preload", group="Robot")
public class A00Preload extends BaseAuto25 {
    @Override
    protected void auto(){
        leftCannon.cannonIntake(true);
        rightCannon.cannonIntake(true);
        sleep(10000);
        leftCannon.spinDown();
        rightCannon.spinDown();
        // chassis.
    }
}