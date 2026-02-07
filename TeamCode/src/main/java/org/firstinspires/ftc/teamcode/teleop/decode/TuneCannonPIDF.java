package org.firstinspires.ftc.teamcode.teleop.decode;
import static org.firstinspires.ftc.teamcode.util.Position.headingFromRelativePosition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.devices.decode.DecodeDevices;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpCannon;

@TeleOp(name="TuneCannonPIDF", group="Linear Opmode")

public class TuneCannonPIDF extends BaseTeleOpDecode {
    String selectedCannonName;
    ScorpCannon sc = null;

    double kp = 0;
    double ki = 0;
    double kd = 0;
    double kf = 0;

    @Override
    protected void teleInit() {
        super.teleInit();
        selectedCannonName = "left";
        sc = devices.leftCannon;
    }

    @Override
    protected void teleIteration() {
        PIDFCoefficients c = sc.getPIDFCoeffs();
        double increment = 0.5;

        kp = c.p;
        ki = c.i;
        kd = c.d;
        kf = c.f;

        // kp: DPAD right+/left-
        // kf: DPAD up+/down-
        // kd: y+/x-
        // ki: b+/a-

        // left 50 0 15 15
        // right 100 0 25 18



        if (intakeReverse) {
            kp += increment;
        } else if (intakeOn) {
            kp -= increment;
        }

        if (intakeEmergency) {
            kf += increment;
        } else if (intakeOff) {
            kf -= increment;
        }

        if (leftCannonMorePower) {
            kd += increment;
        } else if (leftCannonLessPower) {
            kd -= increment;
        }

        if (rightCannonMorePower) {
            ki += increment;
        } else if (rightCannonLessPower) {
            ki -= increment;
        }

        // select
        if (fireLeft) {
            sc = devices.leftCannon;
            selectedCannonName = "Left";
            devices.leftCannon.spinUp();
            devices.rightCannon.spinDown();
        }
        if (fireRight) {
            sc = devices.rightCannon;
            selectedCannonName = "Right";
            devices.rightCannon.spinUp();
            devices.leftCannon.spinDown();
        }

        if (slower) {
            sc.spinDown();
        } else if (faster) {
            sc.spinUp();
        }

        c = new PIDFCoefficients(kp, ki, kd, kf);
        sc.setPIDFCoeffs(c);

        telemetry.clearAll();
        telemetry.addLine("Cannon: "+selectedCannonName);
        telemetry.addData("PIDF current", "%.2f %.2f %.2f %.2f", kp, ki, kd, kf);
        telemetry.addData("Speed", "%.2f of %.2f",
                sc.getPower(), sc.getPowerLevel());
    }
}