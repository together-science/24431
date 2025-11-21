package org.firstinspires.ftc.teamcode.gm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto25;
import org.firstinspires.ftc.teamcode.ScorpCannonMT;
import org.firstinspires.ftc.teamcode.ScorpChassis;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "GMAprilTag", group="Robot")
public class GMAprilTag extends BaseAuto25 {
    @Override
    protected void auto(){
        List<Integer> tags = Arrays.asList(21,22,23);
        AprilTagDetection tag;
        int id = 0;

        tag = camera.getDetection(tags);
        if (tag != null) {
            id = tag.id;
        }
        leftCannon.spinUp();
        rightCannon.spinUp();
        chassis.strafeTo(0, 62, ScorpChassis.DRIVE_SPEED_FAST);
        if (tag == null) {
            // look again
            tag = camera.getDetection(tags);
            if (tag != null) {
                id = tag.id;
            }
        }
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 45);
        fire(id);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 0);
        chassis.strafeTo(0, 0, ScorpChassis.DRIVE_SPEED_FAST);
    }


    private void fire(int tagId) {
        /*   Tag             RAMP MOTIF
                             Index   1    2    3    4    5    6    7    8    9
             21 GPP          GATE  [ G ][ P ][ P ][ G ][ P ][ P ][ G ][ P ][ P ]  SQUARE
             22 PGP          GATE  [ P ][ G ][ P ][ P ][ G ][ P ][ P ][ G ][ P ]  SQUARE
             23 PPG          GATE  [ P ][ P ][ G ][ P ][ P ][ G ][ P ][ P ][ G ]  SQUARE
         */
        // assumption: left cannon = preloaded purple, right cannon = preloaded green

        ScorpCannonMT purple = leftCannon;
        ScorpCannonMT green = rightCannon;

        switch (tagId) {
            case 21:
                green.fire();
                sleep(2500); // pause to let first ball find ramp
                purple.fire();
                break;
            case 22:
                purple.fire();
                sleep(2500); // pause to let first ball find ramp
                green.fire();
                break;
            case 23:
            default:
                // didn't see the tag, or it's asking for 2 purple first, which we don't have
                // doesn't really matter what we do here. No reason to pause between, either
                purple.fire();
                green.fire();
                break;
        }
        sleep(1000); // pause to let fire routine complete (separate thread)
    }
}