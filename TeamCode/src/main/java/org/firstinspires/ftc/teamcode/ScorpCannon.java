package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScorpCannon {
    private DcMotor wheel = null;
    private Servo trigger = null;
    private static final double SPEED = 1.0;
    private static final double POSITION_CHARGED = 0.0;
    private static final double POSITION_TRIGGERED = 1.0;
    private static final long SERVO_DELAY = 200;

    ScorpCannon(HardwareMap hm, String wheelName, String triggerName) {
        try {
            this.wheel = hm.get(DcMotor.class, wheelName);
            this.trigger = hm.get(Servo.class, triggerName);
        } catch(Exception ignored) {
        }

        if (wheel != null) {
            // no encoder, coast on zero
            wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // direction is controlled in hardware config, leave this alone!
            wheel.setDirection(DcMotor.Direction.FORWARD);
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
        if (this.wheel == null) {
            return;
        }

        try {
            trigger.setPosition(POSITION_CHARGED);
            Thread.sleep(SERVO_DELAY);
            trigger.setPosition(POSITION_TRIGGERED);
            Thread.sleep(SERVO_DELAY);
        } catch (InterruptedException ignored) {
        } finally {
            trigger.setPosition(POSITION_CHARGED);
        }
    }
}
