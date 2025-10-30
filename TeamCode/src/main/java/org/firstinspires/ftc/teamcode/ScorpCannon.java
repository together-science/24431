package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class ScorpCannon {
    private CRServo wheel = null;
    private Servo trigger = null;
    private final LinearOpMode op;
    private static final double SPEED = 1.0;
    private static final double POSITION_CHARGED = 0.0;
    private static final double POSITION_TRIGGERED = 1.0;
    private static final long SERVO_DELAY = 200;

    ScorpCannon(LinearOpMode op, String wheelName, String triggerName) {
        this.op = op;
        try {
            this.wheel = op.hardwareMap.get(CRServo.class, wheelName);
            op.telemetry.addLine("found wheel");
            this.trigger = op.hardwareMap.get(Servo.class, triggerName);
            op.telemetry.addLine("found trigger");
        } catch(Exception ignored) {
        }

        if (wheel != null) {
        }
        if (trigger != null) {
            trigger.setPosition(POSITION_CHARGED);
        }
    }

    void spinUp() {
        // check if we in fact have a cannon
        if (this.wheel == null) {
            return;
        }

        wheel.setPower(SPEED);
    }

    void spinDown() {
        // check if we in fact have a cannon
        if (this.wheel == null) {
            return;
        }

        // spin down the wheel ...
        wheel.setPower(0);
    }

    void fire() {
        // check if we in fact have a cannon
        if (this.wheel == null || this.trigger == null) {
            return;
        }

        trigger.setPosition(POSITION_CHARGED);
        op.sleep(SERVO_DELAY);
        trigger.setPosition(POSITION_TRIGGERED);
        op.sleep(SERVO_DELAY);
        trigger.setPosition(POSITION_CHARGED);
    }
}
