package org.firstinspires.ftc.teamcode.devices.decode;
import org.firstinspires.ftc.teamcode.ScorpionLinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ScorpCannonServoPort extends ScorpCannon {
    private CRServo wheel = null;

    ScorpCannonServoPort(ScorpionLinearOpMode op, String wheelName, String triggerName, double power, DcMotorSimple.Direction direction) {
        super(op, triggerName, power, direction);
        try {
            this.wheel = op.hardwareMap.get(CRServo.class, wheelName);
            op.tms.add("found wheel");
        } catch(Exception ignored) {
        }

        if (wheel != null) {
            wheel.setDirection(direction);
        }
    }

    protected void setPower(double power) {
        wheel.setPower(power);
    }
    protected void setPowerRaw(double power) {
        wheel.setPower(power);
    }

    protected boolean noWheel() {
        return wheel == null;
    }

    public double getPower() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return 0.0;
        }
        return wheel.getPower();
    }

}
