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

        kp = c.p;
        ki = c.i;
        kd = c.d;
        kf = c.f;

        if (intakeReverse) {
            kp += 0.05;
        } else if (intakeOn) {
            kp -= 0.05;
        }

        if (intakeEmergency) {
            kf += 0.05;
        } else if (intakeOff) {
            kf -= 0.05;
        }

        if (leftCannonMorePower) {
            kd += 0.05;
        } else if (leftCannonLessPower) {
            kd -= 0.05;
        }

        if (rightCannonMorePower) {
            ki += 0.05;
        } else if (rightCannonLessPower) {
            ki -= 0.05;
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