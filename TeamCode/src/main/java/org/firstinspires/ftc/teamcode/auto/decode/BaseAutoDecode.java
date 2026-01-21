package org.firstinspires.ftc.teamcode.auto.decode;
import org.firstinspires.ftc.teamcode.devices.decode.DecodeDevices;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpCannon;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;

public abstract class BaseAutoDecode extends BaseAuto {

    protected DecodeDevices devices = null;
    int id = 0; // tag id on obelisk


    @Override
    protected void autoInit() {
        super.autoInit();
        devices = new DecodeDevices(this);
    }

    @Override
    protected void autoStart() {
        super.autoStart();
        devices.leftCannon.reset();
        devices.rightCannon.reset();
    }

    protected void getObeliskId() {
        // only if we don't have one
        if (id == 0) {
            //sleep(500); // give the camera a chance to see stuff
            // look (again)
            AprilTagDetection tag;
            tag = chassis.camera.getDetection(Arrays.asList(21,22,23));
            if (tag != null) {
                id = tag.id;
                chassis.camera.off();
            }
        }
    }


    protected void fire() {
        /*   Tag             RAMP MOTIF
                             Index   1    2    3    4    5    6    7    8    9
             21 GPP          GATE  [ G ][ P ][ P ][ G ][ P ][ P ][ G ][ P ][ P ]  SQUARE
             22 PGP          GATE  [ P ][ G ][ P ][ P ][ G ][ P ][ P ][ G ][ P ]  SQUARE
             23 PPG          GATE  [ P ][ P ][ G ][ P ][ P ][ G ][ P ][ P ][ G ]  SQUARE
         */
        // assumption: left cannon = preloaded purple, right cannon = preloaded green

        ScorpCannon purple = devices.leftCannon;
        ScorpCannon green = devices.rightCannon;

        switch (this.id) {
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
