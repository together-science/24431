package org.firstinspires.ftc.teamcode.devices.decode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private final String triggerName;

    ScorpCannon(LinearOpMode op, String triggerName, double power, DcMotorSimple.Direction direction) {
        this.op = op;
        this.power = power;
        this.triggerName = triggerName;
        try {
            this.trigger = op.hardwareMap.get(Servo.class, triggerName);
            op.telemetry.addLine("found trigger "+triggerName);
        } catch(Exception ignored) {
        }

        if (direction == DcMotorSimple.Direction.REVERSE) {
            trigger.setDirection(Servo.Direction.FORWARD);
        } else {
            trigger.setDirection(Servo.Direction.REVERSE);
        }
    }

    abstract protected void setPower(double power);

    void setPowerLevel(double power){
        this.power = power;
    }

    abstract protected boolean noWheel();

    void reset() {
        this.trigger.setPosition(0.0);
        setPower(0);
    }

    void spinUp() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return;
        }

        setPower(power);
        spinDown = true;
    }

    void cannonIntake(boolean stayOn){
        if(noWheel()){
            return;
        }
        //op.telemetry.addLine("intake "+triggerName);
        //op.telemetry.update();
        setPower(-0.30);

        if (stayOn) {
            return;
        }
        spinDown = true;
        lastFired = System.currentTimeMillis();
    }

    void cannonIntake() {
        cannonIntake(false);
    }

    void cannonIntakeEmergencyPower() {
        if(noWheel()){
            return;
        }
        setPower(-0.50);
        op.sleep(1000);
        setPower(0.0);
    }


    void spinDown() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return;
        }

        // spin down the wheel ...
        setPower(0);
        spinDown = false;
    }

    void spinDownAndReleaseAfterDelay() {
        long now = System.currentTimeMillis();
        if(spinDown) {
            if (now - lastFired > TIMEOUT) {
                spinDown();
            }
        }

        if (now - lastFired > SERVO_DELAY) {
            // if statement is here in case we ever have different values
            if (triggerName.equals("right_cannon_trigger")) {
                trigger.setPosition(0.0);
            } else {
                trigger.setPosition(0.0);
            }
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

    double getPowerLevel() {
        return power;
    }

    abstract protected double getPower();


    void fire() {
        //op.telemetry.addLine("Fire method "+triggerName);
        //op.telemetry.update();
        // if necessary, spin up cannon
        if (!noWheel() && getPower() == 0) {
            spinUp();
            //op.telemetry.addLine("Fire method waiting for spinUp noWheel = "+noWheel());
            //op.telemetry.update();
            op.sleep(SPINUP_DELAY);
        }

        // check if we in fact have a cannon
        if (this.trigger == null) {
            //op.telemetry.addLine("No trigger");
            //op.telemetry.update();
            return;
        }

        //op.telemetry.addLine("Firing trigger "+triggerName);
        //op.telemetry.update();

        // determine servo fire position
        double firePosition;
        double restPosition;
        if (triggerName.equals("right_cannon_trigger")) {
            firePosition = 0.4;
            restPosition = 0.0;
        } else {
            firePosition = 0.5;
            restPosition = 0.0;
        }


        // actuate trigger servo
        trigger.setPosition(restPosition);
        trigger.setPosition(firePosition);
//        op.telemetry.addLine("Servo to fire");
//        op.telemetry.update();

        //op.sleep(SERVO_DELAY);
        //trigger.setPosition(restPosition);
//        op.telemetry.addLine("Servo to rest");
//        op.telemetry.update();

        lastFired = System.currentTimeMillis();
        spinDown = true;
    }
}
