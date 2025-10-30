package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class ScorpCannon {
    private CRServo wheel = null;
    private Servo trigger = null;
    private final LinearOpMode op;
    private long lastFired = 0L;
    private static final double SPEED = 1.0;
    private static final double POSITION_CHARGED = 0.0;
    private static final double POSITION_TRIGGERED = 1.0;
    private static final long SERVO_DELAY = 200;
    private static final long SPINUP_DELAY = 2000;
    private static final long TIMEOUT = 10000;

    ScorpCannon(LinearOpMode op, String wheelName, String triggerName) {
        this.op = op;
        try {
            this.wheel = op.hardwareMap.get(CRServo.class, wheelName);
            op.telemetry.addLine("found wheel");
            this.trigger = op.hardwareMap.get(Servo.class, triggerName);
            op.telemetry.addLine("found trigger");
        } catch(Exception ignored) {
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
        lastFired = System.currentTimeMillis();
    }

    void spinDown() {
        // check if we in fact have a cannon
        if (this.wheel == null) {
            return;
        }

        // spin down the wheel ...
        wheel.setPower(0);
    }

    void spinDownAfterDelay() {
        long now = System.currentTimeMillis();
        if (now - lastFired > TIMEOUT) {
            spinDown();
        }
    }

    void fire() {
        // check if we in fact have a cannon
        if (this.wheel == null) {
            return;
        }

        // if necessary, spin up cannon
        if (wheel.getPower() == 0) {
            spinUp();
            op.sleep(SPINUP_DELAY);
        }

        // check if we in fact have a cannon
        if (this.trigger == null) {
            return;
        }

        // actuate trigger servo
        trigger.setPosition(POSITION_CHARGED);
        op.sleep(SERVO_DELAY);
        trigger.setPosition(POSITION_TRIGGERED);
        op.sleep(SERVO_DELAY);
        trigger.setPosition(POSITION_CHARGED);
    }
}
