package org.firstinspires.ftc.teamcode.devices.decode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ScorpCannonMotorPort extends ScorpCannon {
    private DcMotorEx wheel = null;
    PIDFCoefficients c;

    ScorpCannonMotorPort(LinearOpMode op, String wheelName, String triggerName, double power,
                         DcMotorSimple.Direction direction, PIDFCoefficients c) {
        super(op, triggerName, power, direction);
        try {
            this.wheel = op.hardwareMap.get(DcMotorEx.class, wheelName);
            this.wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.wheel.setVelocity(0);
            this.c = c;
        } catch(Exception ignored) {
        }

        if (wheel != null) {
            wheel.setDirection(direction);
        }
    }

    @Override
    public void setPIDFCoeffs(PIDFCoefficients c) {
        wheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
    }

    @Override
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

}
