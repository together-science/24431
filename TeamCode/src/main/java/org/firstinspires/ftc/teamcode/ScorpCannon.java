package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScorpCannon {
    private final DcMotor wheel;
    private final Servo trigger;

    ScorpCannon(HardwareMap hm, String wheelName, String triggerName) {
        this.wheel = hm.get(DcMotor.class, wheelName);
        this.trigger = hm.get(Servo.class, triggerName);
    }

    void spinUp() {
        // check if we in fact have a cannon
        if (this.wheel == null) {
            return;
        }

        // spin up the wheel ...
    }
}
