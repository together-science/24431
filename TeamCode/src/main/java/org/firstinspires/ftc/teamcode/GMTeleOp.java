package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GM TeleOp", group="Linear Opmode")

public class GMTeleOp extends BaseTele {

    @Override
    protected void tele() {
        double currentHeading;
        String intakeState = "off";

        ActionButton fireLeftActionButton = new ActionButton(()->gamepad1.left_bumper);
        ActionButton fireRightActionButton = new ActionButton(()->gamepad1.right_bumper);
        ActionButton intakeOnActionButton = new ActionButton(()->gamepad1.dpad_left);
        ActionButton intakeReverseActionButton = new ActionButton(()->gamepad1.dpad_right);
        ActionButton intakeOffActionButton = new ActionButton(()->gamepad1.dpad_down);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // gather gamepad info
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double yaw = gamepad1.right_stick_x;
            boolean faster = gamepad1.right_trigger > 0.1;
            boolean slower = gamepad1.left_trigger > 0.1;
            boolean fireLeft = fireLeftActionButton.getStatus();
            boolean fireRight = fireRightActionButton.getStatus();
            boolean intakeOn = intakeOnActionButton.getStatus();
            boolean intakeReverse = intakeReverseActionButton.getStatus();
            boolean intakeOff = intakeOffActionButton.getStatus();

            // compute what the intakeState should be
            if (intakeOff) {
                intakeState = "off";
            } else if (intakeOn) {
                intakeState = "on";
            } else if (intakeReverse) {
                intakeState = "reverse";
            }

            // compute faster/slower
            double speedFactor = 0.5;
            if (faster) {
                speedFactor = 1.0; // todo: tune this
            } else if (slower) {
                speedFactor = 0.2; // todo: tune this
            }

            // compute speeds and directions and headings
            double direction = Math.atan2(y,x)/Math.PI*180;
            currentHeading = chassis.getHeading();
            double turnSpeed = yaw*speedFactor;
            double driveSpeed = Math.min(Math.sqrt(x*x + y*y), 1.0)*speedFactor;

            // and drive
            if (driveSpeed > 0.05) {
                chassis.startStrafe(driveSpeed, direction, turnSpeed);
            } else if (Math.abs(turnSpeed) > 0.05) {
                chassis.startTurn(turnSpeed);
            } else {
                chassis.stop();
            }
            // set the intake
            if (intakeState.equals("on")) {
                intake.on();
            } else if (intakeState.equals("reverse")) {
                intake.reverse();
            } if (intakeState.equals("off")) {
                intake.off();
            }

            // fire the cannons!
            if (fireLeft) {
                leftCannon.fire();
            }
            if (fireRight) {
                rightCannon.fire();
            }

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current heading", "%.2f", currentHeading);
            telemetry.addData("turnSpeed", "%.2f", turnSpeed);
            telemetry.update();
        }
    }
}
