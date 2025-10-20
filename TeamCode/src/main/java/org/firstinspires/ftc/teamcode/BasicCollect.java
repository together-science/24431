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

        chassis.moveTo(-6, 3, ScorpChassis.DRIVE_SPEED_NORMAL); // This is us moving to a spike
        intake.on();

        // <Theoretical senario where the code works perfectly> //
        sorter.hold();
        // Pretend there was a color check and it was green
        sorter.sortLeft();
        sorter.hold();
        // Pretend there was a color check and it was purple
        sorter.sortRight();
        sorter.hold();
        // Pretend we grabbed a purple one

        intake.off();

        shootInOrder();
    }

    void shootInOrder(){
        if(green == 1){
            leftCannon.fire();
            sleep(SLEEP_TIME);
            sorter.sortLeft();
            rightCannon.fire();
            sleep(SLEEP_TIME);
            rightCannon.fire();
        }
        if(green == 2){
            rightCannon.fire();
            sleep(SLEEP_TIME);
            sorter.sortLeft();
            leftCannon.fire();
            sleep(SLEEP_TIME);
            rightCannon.fire();
        }
        if(green == 3){
            rightCannon.fire();
            sleep(SLEEP_TIME);
            sorter.sortLeft();
            rightCannon.fire();
            sleep(SLEEP_TIME);
            leftCannon.fire();
        }
    }
}