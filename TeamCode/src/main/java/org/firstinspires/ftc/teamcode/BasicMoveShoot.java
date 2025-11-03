package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
This is Matthew's auto, dont change it.
I know it hardly works, I am working on it possibly.
Making WORKING code was not an expected requirement.
How do I have so much to do and at the same time have nothing to do?
hoursSpent = 1;
 */

@Autonomous(name = "BasicMoveShoot-MM", group = "Sensor")
public class BasicMoveShoot extends BaseAuto{
    int green;
    long SLEEP_TIME = 300;

    @Override
    protected void autoInit(){
//        if(camera.getDetection(21) != null){
//            green = 1;
//        }
//        if(camera.getDetection(22) != null){
//            green = 2;
//        }
//        if(camera.getDetection(23) != null){
//            green = 3;
//        }
    }

    @Override
    protected void auto(){
        chassis.moveTo(0, 12, ScorpChassis.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassis.TURN_SPEED, 45);

        shootInOrder();

        chassis.moveTo(-6, 0, ScorpChassis.DRIVE_SPEED_NORMAL);
    }

    void shootInOrder(){
//        if(green == 1){
//            leftCannon.fire();
//            sleep(SLEEP_TIME);
//            sorter.sortLeft();
//            rightCannon.fire();
//            sleep(SLEEP_TIME);
//            rightCannon.fire();
//        }
//        if(green == 2){
//            rightCannon.fire();
//            sleep(SLEEP_TIME);
//            sorter.sortLeft();
//            leftCannon.fire();
//            sleep(SLEEP_TIME);
//            rightCannon.fire();
//        }
//        if(green == 3){
//            rightCannon.fire();
//            sleep(SLEEP_TIME);
//            sorter.sortLeft();
//            rightCannon.fire();
//            sleep(SLEEP_TIME);
//            leftCannon.fire();
//        }
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