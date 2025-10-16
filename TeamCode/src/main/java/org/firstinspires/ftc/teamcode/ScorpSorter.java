package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScorpSorter {
    private Servo servo = null;
    private static final double POSITION_HOLD = 0.0;
    private static final double POSITION_LEFT = 1.0;
    private static final double POSITION_RIGHT = -1.0;

    ScorpSorter(HardwareMap hm, String servoName){
        try {
            servo = hm.get(Servo.class, servoName);
        } catch(Exception ignored) {
        }
    }

    void hold() {
        servo.setPosition(POSITION_HOLD);
    }

    void sortLeft() {
        servo.setPosition(POSITION_LEFT);
    }

    void sortRight() {
        servo.setPosition(POSITION_RIGHT);
    }
}
