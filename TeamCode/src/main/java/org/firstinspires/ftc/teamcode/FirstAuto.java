package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
This is Matthew's auto, dont change it.
I know it hardly works, I am working on it possibly.
Making WORKING code was not an expected requirement.
hoursSpent = 1;
 */

@Autonomous(name = "AutoTest-MM", group = "Sensor")
public class FirstAuto extends BaseAuto{
    boolean firstColor;
    boolean secondColor;
    boolean thirdColor;
    boolean green = false;
    boolean purple = true;

    @Override
    protected void autoInit(){
        if(camera.getDetection(21) != null){
            firstColor = green; secondColor = purple; thirdColor = purple;
        }
        if(camera.getDetection(22) != null){
            firstColor = purple; secondColor = green; thirdColor = purple;
        }
        if(camera.getDetection(23) != null){
            firstColor = purple; secondColor = purple; thirdColor = green;
        }
    }

    @Override
    protected void auto(){
        chassis.moveTo(0, 12, ScorpChassis.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassis.TURN_SPEED, 45);

        shootInOrder();

        chassis.moveTo(-6, 0, ScorpChassis.DRIVE_SPEED_NORMAL);
    }

    void shootInOrder(){
        if(firstColor) leftCannon.fire();
        else rightCannon.fire();
        if(secondColor) leftCannon.fire();
        else rightCannon.fire();
        if(thirdColor) leftCannon.fire();
        else rightCannon.fire();
    }
}

/* Visualization of whats going down
--------------------------------------
|  /             /_\ <-- Code        |
| /   <-- Goal                       |
|/                                   |
|                                    |
|                                    |
|                                    |
|                                    |
|                                    |
|                 **  <-- Robot      |
--------------------------------------

--------------------------------------
|  /             /_\                 |
| /                                  |
|/              \  /   <-- Scan code |
|                **                  |
|                                    |
|                                    |
|                                    |
|                                    |
|                                    |
--------------------------------------

--------------------------------------
|  /             /_\                 |
| /                                  |
|/        *                          |
|        *                           |
|                                    |
|                                    |
|                                    |
|                                    |
|                                    |
--------------------------------------

--------------------------------------
|  /             /_\                 |
| /    \ <-- Projectile              |
|/        *     In                   |
|        *       Order               |
|                                    |
|                                    |
|                                    |
|                                    |
|                                    |
--------------------------------------
*/