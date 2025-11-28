package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOtos;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpCannon;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red01AT", group="Robot")
public class Red01AT extends BaseAutoDecode {
    @Override
    protected void auto(){
        devices.camera.off();
        chassis.strafeTo(0, -45, ScorpChassisOtos.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, 45);
        devices.camera.on();
        sleep(500);
        List<Integer> tags = Arrays.asList(21,22,23);
        AprilTagDetection tag;
        int id = 0;
        tag = devices.camera.getDetection(tags);
        if (tag != null) {
            id = tag.id;
        }
        sleep(500);
        devices.leftCannon.spinUp();
        devices.rightCannon.spinUp();
        if (tag == null) {
            // look again
            tag = devices.camera.getDetection(tags);
            if (tag != null) {
                id = tag.id;
            }
        }
        devices.camera.on();
        sleep(500);
        chassis.turnToHeading(0.5, 0);
        fire(id);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, 45);
        chassis.strafeTo(11, -65, ScorpChassisOtos.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassisOtos.DRIVE_SPEED_NORMAL, 145);
    }


    private void fire(int tagId) {
        /*   Tag             RAMP MOTIF
                             Index   1    2    3    4    5    6    7    8    9
             21 GPP          GATE  [ G ][ P ][ P ][ G ][ P ][ P ][ G ][ P ][ P ]  SQUARE
             22 PGP          GATE  [ P ][ G ][ P ][ P ][ G ][ P ][ P ][ G ][ P ]  SQUARE
             23 PPG          GATE  [ P ][ P ][ G ][ P ][ P ][ G ][ P ][ P ][ G ]  SQUARE
         */
        // assumption: left cannon = preloaded purple, right cannon = preloaded green

        ScorpCannon purple = devices.leftCannon;
        ScorpCannon green = devices.rightCannon;

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