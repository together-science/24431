package org.firstinspires.ftc.teamcode.teleop.biobuzz;
import static org.firstinspires.ftc.teamcode.util.Position.headingFromRelativePosition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.decode.BaseTeleOpDecode;

import java.util.Arrays;

@TeleOp(name="Amarra BioBuzz", group="BioBuzz")

public class Amarra extends BaseTeleOpDecode {
    double heading = 0;
    boolean rumble = false;
    @Override
    protected void teleIteration() {
        // compute what the intakeState should be
        if (intakeOff) {
            devices.intake.on();
        } else if (intakeOn) {
            devices.intake.on();
        } else if (intakeReverse) {
            devices.intake.off();
        }

        // compute faster/slower
        double speedFactor = 0.5;
        if (faster) {
            speedFactor = 1.0; // todo: tune this
        } else if (slower) {
            speedFactor = 0.2; // todo: tune this
        }

        // adjust cannon wheel speed
        if (launcherMorePower) {
            devices.launcher.morePower();
        } else if (launcherLessPower) {
            devices.launcher.lessPower();
        }
        if (rightCannonMorePower) {
            devices.rightCannon.morePower();
        } else if (rightCannonLessPower) {
            devices.rightCannon.lessPower();
        }

        // compute speeds and directions and headings
        double direction = headingFromRelativePosition(x,y);
        currentHeading = chassis.getIMUHeading();
        double turnInput = Math.abs(yaw);
        double turnWhileStrafingSpeed = yaw*speedFactor;
        double turnSpeed = yaw*speedFactor;
        double driveInput = Math.sqrt(x*x + y*y);
        double driveSpeed = Math.min(driveInput, 1.0)*speedFactor;
        boolean forward = Math.abs(direction) < 90;

        // and drive
        if (driveInput > 0.05) {
            if (turnInput > 0.05 && forward) {
                // forward turn
                chassis.startStrafe(driveSpeed, direction, turnWhileStrafingSpeed, chassis.getIMUHeading());
                heading = chassis.getIMUHeading();
            } else if (turnInput > 0.05) {
                // backward turn
                chassis.startStrafe(driveSpeed, direction, -turnWhileStrafingSpeed, chassis.getIMUHeading());
                heading = chassis.getIMUHeading();
            } else {
                // straight
                chassis.startStrafe(driveSpeed, direction, 0, heading);
            }
        } else if (turnInput > 0.05) {
            chassis.startTurn(turnSpeed);
            heading = chassis.getIMUHeading();
        } else {
            chassis.stopDrive();
            heading = chassis.getIMUHeading();
        }

        // fire the cannons!
        boolean autoAim = false;
        if (fire) {
            chassis.autoAim(Arrays.asList(20, 24));
            autoAim = true;
        }

        double lc = devices.launcher.getPower();
        double lcl = devices.launcher.getPowerLevel();
        if (rumble && lc >= lcl-0.01) {
            gamepad1.rumble(100);
            rumble = false;
        }

        if (fire) {
            devices.launcher.fire();
            rumble = false;
        }

        // add the elapsed game time
        telemetry.addData("Current heading", "%.2f", currentHeading);
        telemetry.addData("x position:", chassis.getPosition().x);
        telemetry.addData("y position:", chassis.getPosition().y);
        telemetry.addData("yaw", "%.2f", yaw);
        telemetry.addData("turnSpeed", "%.2f", turnSpeed);
        telemetry.addData("Launcher", "%.2f of %.2f",
                devices.launcher.getPower(), devices.launcher.getPowerLevel());
        telemetry.addData("trigger", fire);
        double angle = chassis.getGoalDetection();
        telemetry.addLine("Detections: "+(angle < 1000?Double.toString(angle):"--------"));
    }
}