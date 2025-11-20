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
            op.telemetry.addLine("found trigger " + triggerName);
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
    public void reset() {
        this.trigger.setPosition(0.0);
        setPower(0);
    }
    public void spinUp() {
        if (this.noWheel()) {
            return;
        }
        setPower(power);
    }
    void cannonIntake() {
        cannonIntake(false);
    }
    void cannonIntake(boolean stayOn){
        if(noWheel()){
            return;
        }
        setPower(-0.30);
        if (!stayOn) {
            spinUpAfterDelay();
        }
    }
    void cannonIntakeEmergencyPower() {
        if(noWheel()){
            return;
        }
        setPower(-1.0);
        op.sleep(1000);
        setPower(0.0);
    }
    void spinDown() {
        if (this.noWheel()) {
            return;
        }
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
        if (this.noWheel()) {
            return;
        }
        power = Math.min(power + 0.05, 1.0);
        if (Math.abs(getPower()) > 0) {
            spinUp(); // Ask drivers if they would like cannon to start when they change power
        }
    }
    void lessPower() {
        if (this.noWheel()) {
            return;
        }
        power = Math.max(power -0.05, 0.05);
        if (Math.abs(getPower()) > 0) {
            spinUp(); // Ask drivers if they would like cannon to start when they change power
        }
    }
    double getPowerLevel() {
        return power;
    }
    abstract protected double getPower();


    public void fire() {
        if (!noWheel() && getPower() == 0) {
            new Thread(()->{
                spinUp();
                op.sleep(SPINUP_DELAY);
                fire();
            }).start();
            return;
        }
        if (this.trigger == null) {
            return;
        }
        double firePosition;
        double restPosition;
        if (triggerName.equals("right_cannon_trigger")) {
            firePosition = 0.5;
        } else {
            firePosition = 0.5;
        }
        restPosition = 0.0;
        trigger.setPosition(restPosition);
        trigger.setPosition(firePosition);
        new Thread(()->{
            op.sleep(SERVO_DELAY);
            trigger.setPosition(restPosition);
            spinDownAfterDelay();
        }).start();
    }
}
