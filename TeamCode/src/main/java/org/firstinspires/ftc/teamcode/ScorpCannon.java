package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ScorpCannon {
    private Servo trigger = null;
    private final LinearOpMode op;
    private long lastFired = 0L;
    private double power;
    private boolean spinDown = false;
    private static final double POSITION_CHARGED = 0.0;
    private static final double POSITION_TRIGGERED = 1.0;
    private static final long SERVO_DELAY = 500;
    private static final long SPINUP_DELAY = 2000;
    private static final long TIMEOUT = 3000;

    ScorpCannon(LinearOpMode op, String triggerName, double power) {
        this.op = op;
        this.power = power;
        try {
            op.telemetry.addLine("found wheel");
            this.trigger = op.hardwareMap.get(Servo.class, triggerName);
            op.telemetry.addLine("found trigger");
        } catch(Exception ignored) {
        }

        if (trigger != null) {
            trigger.setPosition(POSITION_CHARGED);
        }
    }

    abstract protected void setPower(double power);

    abstract protected boolean noWheel();

    void spinUp() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return;
        }

        setPower(power);
        lastFired = System.currentTimeMillis();
    }

    void cannonIntake(){
        if(noWheel()){
            return;
        }
        spinDown = true;
        setPower(-0.40);
        lastFired = System.currentTimeMillis();
    }

    void spinDown() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return;
        }

        // spin down the wheel ...
        setPower(0);
        setPower(0);
        spinDown = false;
    }

    void spinDownAfterDelay() {
        if(!spinDown){
            return;
        }
        long now = System.currentTimeMillis();
        if (now - lastFired > TIMEOUT && !spinDown) {
            spinDown();
        }
    }

    void morePower() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return;
        }

        // increase by 5%
        power = Math.min(power +0.05, 1.0);

        if (Math.abs(getPower()) > 0) {
            // spinning, adjust to new speed
            spinUp();
        }
    }

    void lessPower() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return;
        }

        // decrease by 5%
        power = Math.max(power -0.05, 0.05);

        if (Math.abs(getPower()) > 0) {
            // spinning, adjust to new speed
            spinUp();
        }
    }

    abstract protected double getPower();


    void fire() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return;
        }
        spinDown = true;

        // if necessary, spin up cannon
        if (getPower() == 0) {
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
