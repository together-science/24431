package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ScorpCannonMT {
    private static final long SERVO_DELAY = 500;
    private static final long SPINUP_DELAY = 2000;
    private static final long SPINDOWN_AFTER_DELAY = 2000;
    private static final long SPINUP_AFTER_INTAKE_DELAY = 6000;

    private Servo trigger = null;
    private final LinearOpMode op;
    private double power;
    private final String triggerName;
    private Thread spinDownThread = null;
    private Thread spinUpThread = null;

    ScorpCannonMT(LinearOpMode op, String triggerName, double power, DcMotorSimple.Direction direction) {
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
    }

    void cannonIntake(boolean stayOn){
        if(noWheel()){
            return;
        }
        //op.telemetry.addLine("intake "+triggerName);
        //op.telemetry.update();
        setPower(-0.30);

        if (!stayOn) {
            spinUpAfterDelay();
        }
    }

    void cannonIntakeEmergencyPower() {
        if(noWheel()){
            return;
        }
        setPower(-0.50);
        op.sleep(1000);
        setPower(0.0);
    }

    void cannonIntake() {
        cannonIntake(false);
    }

    void spinDown() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return;
        }

        // spin down the wheel ...
        setPower(0);
    }

    void spinDownAfterDelay() {
        if (spinDownThread != null) {
            spinDownThread.interrupt();
        }
        spinDownThread = new Thread(()->{
            try {
                Thread.sleep(SPINDOWN_AFTER_DELAY);
            } catch (InterruptedException e) {
                return;
            }
            spinDown();
            spinDownThread = null;
        });
        spinDownThread.start();
    }

    void spinUpAfterDelay() {
        // the intake is on. after intake delay, spin up cannon for fire
        if (spinUpThread != null) {
            spinUpThread.interrupt();
        }
        spinUpThread = new Thread(()->{
            try {
                Thread.sleep(SPINUP_AFTER_INTAKE_DELAY);
            } catch (InterruptedException e) {
                return;
            }
            spinUp();
            spinUpThread = null;
        });
        spinUpThread.start();
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
        // if necessary, spin up cannon
        if (!noWheel() && getPower() == 0) {
            // in that case, the whole thing happens in a new thread
            new Thread(()->{
                spinUp();
                op.sleep(SPINUP_DELAY);
                fire();
            }).start();
            return;
        }

        // check if we in fact have a cannon
        if (this.trigger == null) {
            return;
        }

        // determine servo fire position
        double firePosition;
        double restPosition;
        if (triggerName.equals("right_cannon_trigger")) {
            firePosition = 0.4;
            restPosition = 0.0;
        } else {
            firePosition = 0.5;
            restPosition = 0.0;
        };


        // actuate trigger servo
        trigger.setPosition(restPosition);
        trigger.setPosition(firePosition);

        new Thread(()->{
            op.sleep(SERVO_DELAY);
            trigger.setPosition(restPosition);
            spinDownAfterDelay();
        }).start();
    }
}
