package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScorpSorter {
    private final Servo servo;

    ScorpSorter(HardwareMap hm, String servoName){
        servo = hm.get(Servo.class, servoName);
    }
}
