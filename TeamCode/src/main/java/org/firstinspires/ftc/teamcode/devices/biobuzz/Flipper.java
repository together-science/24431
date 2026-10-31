package org.firstinspires.ftc.teamcode.devices.biobuzz;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

public class Flipper {
    private Servo flipperServo = null;
    private static final double OPEN = -1.0;
    private static final double CLOSED = 1.0;
    private final BaseTeleOp op;
    private Thread thread;

    Flipper(BaseTeleOp op, String flipperName) {
        this.op = op;
        try {
            flipperServo = op.hardwareMap.get(Servo.class, flipperName);
//            op.tms.add("found flipper servo");
        } catch (Exception ignored) {
        }
    }

    public void open() {
        if (flipperServo == null) {
            return ;
        }

        flipperServo.setPosition(OPEN);
    }

    public void close() {
        if (flipperServo == null) {
            return ;
        }

        flipperServo.setPosition(CLOSED);
    }

    public void on() {
        if (flipperServo == null) {
            return ;
        }

        // thread already up? all done here
        if (thread != null) {
            return;
        }

        thread = new Thread(()->{
            open();
            try {
                do {
                    open();
                    Thread.sleep(1000);
                    close();
                    Thread.sleep(1000);
                } while (true);
            } catch (InterruptedException ex) {
                op.tms.add("Flipper thread exception:"+ex.getMessage());
            }
        });
        thread.start();
        op.tms.add("Flipper on");
    }

    public void off() {
        if (flipperServo == null) {
            return ;
        }

        if (thread == null) {
            return;
        }

        thread.interrupt();
        thread = null;
        close();
        op.tms.add("Flipper off");
    }


    public void toggle() {
        if (flipperServo == null) {
            return ;
        }

        if (thread == null) {
            on();
        } else {
            off();
        }
    }
}
