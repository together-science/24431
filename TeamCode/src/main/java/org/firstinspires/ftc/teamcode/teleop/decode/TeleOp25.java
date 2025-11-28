package org.firstinspires.ftc.teamcode.teleop.decode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp25", group="Linear Opmode")

public class TeleOp25 extends BaseTeleOp25 {
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
            devices.intake.reverse();
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
        double direction = chassis.headingFromRelativePosition(x, y);
        currentHeading = chassis.getHeading();
        double turnInput = Math.abs(yaw);
        double turnWhileStrafingSpeed = yaw*speedFactor*2;
        double turnSpeed = yaw*speedFactor;
        double driveInput = Math.sqrt(x*x + y*y);
        double driveSpeed = Math.min(driveInput, 1.0)*speedFactor;

        // and drive
        if (driveInput > 0.05) {
            chassis.startStrafe(driveSpeed, direction, turnWhileStrafingSpeed);
        } else if (turnInput > 0.05) {
            chassis.startTurn(turnSpeed);
        } else {
            chassis.stop();
        }

        // fire the cannons!
        if (fireLeft) {
            telemetry.addLine("Before calling fire");
            telemetry.update();
            devices.leftCannon.fire();
            telemetry.addLine("After calling fire");
            telemetry.update();
        }
        if (fireRight) {
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
    }
}
