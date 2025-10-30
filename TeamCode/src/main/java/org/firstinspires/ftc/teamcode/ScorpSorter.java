package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ScorpSorter {
    private Servo servo = null;
    private static final double POSITION_HOLD = 0.0;
    private static final double POSITION_LEFT = 1.0;
    private static final double POSITION_RIGHT = -1.0;

    ScorpSorter(LinearOpMode op, String servoName){
        try {
            servo = op.hardwareMap.get(Servo.class, servoName);
        } catch(Exception ignored) {
        }
    }

    void hold() {
        if (servo == null) {
            return;
        }
        servo.setPosition(POSITION_HOLD);
    }

    void sortLeft() {
        if (servo == null) {
            return;
        }
        servo.setPosition(POSITION_LEFT);
    }

    void sortRight() {
        if (servo == null) {
            return;
        }
        servo.setPosition(POSITION_RIGHT);
    }
}
