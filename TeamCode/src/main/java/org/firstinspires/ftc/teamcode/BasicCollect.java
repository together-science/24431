package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BasicCollect-MM", group = "Sensor")
public class BasicCollect extends BaseAuto{
    int green;
    long SLEEP_TIME = 300;

    @Override
    protected void autoInit(){
        if(camera.getDetection(21) != null){
            green = 1;
        }
        if(camera.getDetection(22) != null){
            green = 2;
        }
        if(camera.getDetection(23) != null){
            green = 3;
        }
    }

    @Override
    protected void auto(){
        chassis.moveTo(0, 12, ScorpChassis.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassis.TURN_SPEED, 45);

        shootInOrder();

        intake.on();
//        chassis.moveTo(color.getPurpleXPos(), color.getPurpleYPos(), ScorpChassis.DRIVE_SPEED_FAST);
//        sorter.sortLeft();
//        sorter.hold();
//        chassis.moveTo(color.getGreenXPos(), color.getGreenYPos(), ScorpChassis.DRIVE_SPEED_FAST);
//        sorter.sortRight();
//        sorter.hold();
//        chassis.moveTo(color.getGreenXPos(), color.getGreenYPos(), ScorpChassis.DRIVE_SPEED_FAST);
        intake.off();

        shootInOrder();
    }

    /*
    You lost the game
     */ // Don't look at this if your name is Matthew

    void shootInOrder(){
        if(green == 1){
            leftCannon.fire();
            sleep(SLEEP_TIME);
            rightCannon.fire();
            sorter.sortRight();
            sleep(SLEEP_TIME);
            rightCannon.fire();
        }
        if(green == 2){
            rightCannon.fire();
            sleep(SLEEP_TIME);
            sorter.sortLeft();
            leftCannon.fire();
            sorter.sortRight();
            sleep(SLEEP_TIME);
            rightCannon.fire();
        }
        if(green == 3){
            rightCannon.fire();
            sorter.sortRight();
            sleep(SLEEP_TIME);
            rightCannon.fire();
            sleep(SLEEP_TIME);
            leftCannon.fire();
        }
    }
}