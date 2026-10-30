package org.firstinspires.ftc.teamcode.devices.biobuzz;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

public class Flipper {
    private Servo flipperServo = null;
    private static final double OPEN = 1.0;
    private static final double CLOSED = -1.0;
    private final BaseTeleOp op;

    Flipper(BaseTeleOp op, String flipperName) {
        this.op = op;
        try {
            flipperServo = op.hardwareMap.get(Servo.class, flipperName);
            op.tms.add("found flipper servo");
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
}
