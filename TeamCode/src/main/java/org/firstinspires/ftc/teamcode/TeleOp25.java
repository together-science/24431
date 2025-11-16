package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp25", group="Linear Opmode")

public class TeleOp25 extends BaseTeleOp25 {
    @Override
    protected void teleIteration() {
        // compute what the intakeState should be
        if (intakeOff) {
            leftCannon.spinDown();
            rightCannon.spinDown();
        } else if (intakeOn) {
            intake.on();
            leftCannon.cannonIntake();
            rightCannon.cannonIntake();
        } else if (intakeReverse) {
            intake.reverse();
        } else if (intakeEmergency) {
            leftCannon.cannonIntakeEmergencyPower();
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
            leftCannon.morePower();
        } else if (leftCannonLessPower) {
            leftCannon.lessPower();
        }
        if (rightCannonMorePower) {
            rightCannon.morePower();
        } else if (rightCannonLessPower) {
            rightCannon.lessPower();
        }

        // compute speeds and directions and headings
        double direction = Math.atan2(y,x)/Math.PI*180;
        currentHeading = chassis.getHeading();
        double turnSpeed = yaw*speedFactor;
        double driveSpeed = Math.min(Math.sqrt(x*x + y*y), 1.0)*speedFactor;

        // and drive
        if (driveSpeed > 0.05) {
            chassis.startStrafe(driveSpeed, direction, turnSpeed*50);
        } else if (Math.abs(turnSpeed) > 0.05) {
            chassis.startTurn(turnSpeed);
        } else {
            chassis.stop();
        }

        // fire the cannons!
        if (fireLeft) {
            telemetry.addLine("Before calling fire");
            telemetry.update();
            leftCannon.fire();
            telemetry.addLine("After calling fire");
            telemetry.update();
        }
        if (fireRight) {
            rightCannon.fire();
        }

        // add the elapsed game time
        telemetry.addData("Current heading", "%.2f", currentHeading);
        telemetry.addData("turnSpeed", "%.2f", turnSpeed);
        telemetry.addData("Left cannon", "%.2f of %.2f",
                leftCannon.getPower(), leftCannon.getPowerLevel());
        telemetry.addData("Right cannon", "%.2f of %.2f", rightCannon.getPower(),
                rightCannon.getPowerLevel());
    }
}
