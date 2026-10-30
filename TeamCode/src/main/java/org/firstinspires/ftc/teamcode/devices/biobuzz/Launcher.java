package org.firstinspires.ftc.teamcode.devices.biobuzz;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

public class Launcher {
    private DcMotorEx wheel = null;
    PIDFCoefficients c;

    private static final long SERVO_DELAY = 500;
    private static final long SPINUP_DELAY = 1500;
    private static final long SPINDOWN_AFTER_DELAY = 100;
//    private static final long SPINUP_AFTER_INTAKE_DELAY = 3500;
    private Servo left = null;
    private Servo right = null;
    private BaseTeleOp op;
    private double power;
    private Thread spinDownThread = null;
    private Thread spinUpThread = null;

    Launcher(BaseTeleOp op, String wheelName, String leftName, String rightName,
             double power,
             DcMotorSimple.Direction direction, PIDFCoefficients c) {
        try {
            this.wheel = op.hardwareMap.get(DcMotorEx.class, wheelName);
            this.wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.wheel.setVelocity(power);
            this.c = c;
            this.op = op;
            this.power = power;
            left = op.hardwareMap.get(Servo.class, leftName);
            right = op.hardwareMap.get(Servo.class, rightName);
            if (wheel != null && left != null && right != null) {
                this.op.telemetry.addLine("launcher construction succeeded");
            } else {
                this.op.telemetry.addLine("launcher construction failed");
                this.op.telemetry.update();
                this.op.sleep(2000);
                throw new RuntimeException();
            }
        } catch(Exception ex) {
            this.op.telemetry.addLine("launcher construction failed");
            this.op.telemetry.update();
            this.op.sleep(2000);
            throw ex;
        }

        if (wheel != null) {
            wheel.setDirection(direction);
        }
    }

    public void setPIDFCoeffs(PIDFCoefficients c) {
        wheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
    }

    public PIDFCoefficients getPIDFCoeffs() {
        return this.wheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void setPower(double power) {
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        wheel.setVelocity(power*2000);
    }
    protected void setPowerRaw(double power){
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel.setPower(power);
    }

    protected boolean noWheel() {
        return wheel == null;
    }

    public double getPower() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return 0.0f;
        }
        return wheel.getVelocity() / 2000;
    }


    public void reset() {
        if(left != null) {
            this.left.setPosition(0.0);
            this.right.setPosition(0.0);
            setPower(0.0);
        }
    }
    public void spinUp() {
        if (this.noWheel()) {
            return;
        }
        op.tms.add("Spinning up");
        setPower(power);
    }

    public void spinDown() {
        if (this.noWheel()) {
            return;
        }
        op.tms.add("Spinning down");
        setPower(0);
    }
    void spinDownAfterDelay() {
        if (spinDownThread != null) {
            spinDownThread.interrupt();
        }
        spinDownThread = new Thread(()->{
            op.tms.add("Spin-down thread started");
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
//    void spinUpAfterDelay() {
//        if (spinUpThread != null) {
//            spinUpThread.interrupt();
//        }
//        spinUpThread = new Thread(()->{
//            try {
//                Thread.sleep(SPINUP_AFTER_INTAKE_DELAY);
//            } catch (InterruptedException e) {
//                return;
//            }
//            spinUp();
//            spinUpThread = null;
//        });
//        spinUpThread.start();
//    }
    public void morePower() {
        if (this.noWheel()) {
            return;
        }
        power = Math.min(power + 0.05, 1.0);
        if (Math.abs(getPower()) > 0) {
            spinUp(); // Ask drivers if they would like cannon to start when they change power
        }
    }
    public void lessPower() {
        if (this.noWheel()) {
            return;
        }
        power = Math.max(power -0.05, 0.05);
        if (Math.abs(getPower()) > 0) {
            spinUp(); // Ask drivers if they would like cannon to start when they change power
        }
    }
    public double getPowerLevel() {
        return power;
    }

    public void fire() {
        if (this.left == null || this.noWheel()) {
            throw new RuntimeException();
//            return;
        }
        if (!noWheel() && getPower() == 0) {
            new Thread(()->{
                spinUp();
                op.tms.add("Spinup delay ...");
                op.sleep(SPINUP_DELAY);
                fire();
            }).start();
            return;
        }

        final double firePosition = -1.0;
        final double restPosition = 1.0;
        if(left != null) {
            op.tms.add("Firing servos");
            left.setPosition(firePosition);
            right.setPosition(-firePosition);
        }
        new Thread(()->{
            op.tms.add("Trigger reset thread sleeping");
            op.sleep(SERVO_DELAY);
            if(left != null) {
                op.tms.add("Trigger reset starting");
                left.setPosition(restPosition);
                right.setPosition(-restPosition);
                op.tms.add("Trigger reset complete");
            }
            spinDownAfterDelay();
        }).start();
    }

    public void rawFire() {
        final double firePosition = -1.0;
        final double restPosition = 1.0;
        left.setPosition(restPosition);
        left.setPosition(firePosition);
        this.op.sleep(1000);
        right.setPosition(-restPosition);
        right.setPosition(-firePosition);
    }
}
