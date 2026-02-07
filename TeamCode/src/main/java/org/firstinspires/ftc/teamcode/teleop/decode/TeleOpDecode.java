package org.firstinspires.ftc.teamcode.teleop.decode;
import static org.firstinspires.ftc.teamcode.util.Position.headingFromRelativePosition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name="TeleOpDecode", group="Linear Opmode")

public class TeleOpDecode extends BaseTeleOpDecode {
    double heading = 0;
    @Override
    protected void teleIteration() {
        // compute what the intakeState should be
        if (intakeOff) {
            devices.leftCannon.spinDown();
            devices.rightCannon.spinDown();
        } else if (intakeOn) {
            devices.intake.on();
            devices.leftCannon.cannonIntake();
            devices.rightCannon.cannonIntake();
        } else if (intakeReverse) {
            devices.intake.off();
            devices.leftCannon.spinUp();
            devices.rightCannon.spinUp();
        } else if (intakeEmergency) {
            devices.leftCannon.cannonIntakeEmergencyPower();
            devices.rightCannon.cannonIntakeEmergencyPower();
        }

        // compute faster/slower
        double speedFactor = 0.5;
        if (faster) {
            speedFactor = 1.0; // todo: tune this
        } else if (slower) {
            speedFactor = 0.2; // todo: tune this
        }

        // adjust cannon wheel speed
        if (leftCannonMorePower) {
            devices.leftCannon.morePower();
        } else if (leftCannonLessPower) {
            devices.leftCannon.lessPower();
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
        if (fireLeft) {
            //chassis.autoAim(Arrays.asList(20, 24));
            devices.leftCannon.fire();
        }
        if (fireRight) {
            //chassis.autoAim(Arrays.asList(20, 24));
            devices.rightCannon.fire();
        }

        // add the elapsed game time
        telemetry.addData("Current heading", "%.2f", currentHeading);
        telemetry.addData("x position:", chassis.getPosition().x);
        telemetry.addData("y position:", chassis.getPosition().y);
        telemetry.addData("yaw", "%.2f", yaw);
        telemetry.addData("turnSpeed", "%.2f", turnSpeed);
        telemetry.addData("Left cannon", "%.2f of %.2f",
                devices.leftCannon.getPower(), devices.leftCannon.getPowerLevel());
        telemetry.addData("Right cannon", "%.2f of %.2f", devices.rightCannon.getPower(),
                devices.rightCannon.getPowerLevel());
        telemetry.addData("fireRight", fireRight);
        telemetry.addData("fireLeft", fireLeft);
    }
}