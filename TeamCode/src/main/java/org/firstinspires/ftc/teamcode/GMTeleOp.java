/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="GM TeleOp", group="Linear Opmode")

public class GMTeleOp extends LinearOpMode {

    private ScorpCannon leftCannon = null;
    private ScorpCannon rightCannon = null;
    private ScorpChassis chassis = null;
    private ScorpIntake intake = null;
    private ScorpSorter sorter = null;

    private final ElapsedTime runtime = new ElapsedTime();

    private static final double TURN_INCREMENT = 0.1;

    @Override
    public void runOpMode() {

        chassis = new ScorpChassis(this, hardwareMap, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        leftCannon = new ScorpCannon(hardwareMap, "left_cannon_wheel", "left_cannon_trigger");
        rightCannon = new ScorpCannon(hardwareMap, "right_cannon_wheel", "right_cannon_trigger");
        intake = new ScorpIntake(hardwareMap, "left_intake", "right_intake");
        sorter = new ScorpSorter(hardwareMap, "sorter_servo");

        chassis.init();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
    
        runtime.reset(); // keeping track of game time
        double currentHeading;
        String intakeState = "off";
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // gather gamepad info
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double yaw = gamepad1.right_stick_x;
            boolean faster = gamepad1.right_trigger > 0.1;
            boolean slower = gamepad1.left_trigger > 0.1;
            boolean fireLeft = gamepad1.left_bumper;
            boolean fireRight = gamepad1.right_bumper;
            boolean intakeOn = gamepad1.dpad_left;
            boolean intakeReverse = gamepad1.dpad_right;
            boolean intakeOff = gamepad1.dpad_down;

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
            double desiredHeading = currentHeading + Math.signum(yaw)*TURN_INCREMENT;
            double turnSpeed = Math.abs(yaw)*speedFactor;
            double driveSpeed = Math.min(Math.sqrt(x*x + y*y), 1.0)*speedFactor;

            // and drive
            chassis.startStrafe(driveSpeed, direction, desiredHeading, turnSpeed);

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
            telemetry.update();
        }
    }
}
