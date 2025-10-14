package org.firstinspires.ftc.teamcode;

public class Trigger {
    private long last = 0;
    private boolean triggered = false;

    private static final long DELAY = 200;

    boolean process(boolean keyDown) {
        if (keyDown) {
            if (triggered) {
                // was already triggered, don't do it again
                return false;
            } else {
                if (System.currentTimeMillis() - last > DELAY) {
                    // this is new!
                    // keyDown, not triggered yet => what we want
                    triggered = true;
                    return true;
                } else {
                    return false;
                }
            }
        } else {
            if (triggered) {
                // keyUp, triggered => release, set timer
                triggered = false;
                last = System.currentTimeMillis();
                return false;
            } else {
                return false;
            }
        }
    }
}
