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

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Scorpions TeleOp", group="Linear Opmode")

public class TeleOp1 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm1 = null;
    private DcMotor arm2 = null;
    private DcMotor hanger = null;
    private DcMotor winch = null;
    private Servo grabber = null;
    private Servo wrist = null;
    private Servo grabber2 = null;
    private AnalogInput angle = null;
    
    static final double armPositions[] = new double[]{
        2.5, // init   0
        0.99, // basket 1
        0.80, // chamber2 2
        0.95, // over 3
        0.07, // pick from fence
        0.05, // pickup
        };
       
    static final int ARM_INIT = 0;
    static final int ARM_BASKET = 1;
    static final int ARM_CHAMBER2= 2;
    static final int ARM_UNDER = 3;
    static final int ARM_PICK_FENCE = 4;
    static final int ARM_PICK_FLOOR = 5;
    
    private int armPosition = ARM_INIT;
    private double wp = 0.03;
    private double winchDefault = 0.0;
            

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        hanger = hardwareMap.get(DcMotor.class, "hanger");
        winch = hardwareMap.get(DcMotor.class, "winch");
        grabber = hardwareMap.get(Servo.class, "grabber1");
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber2= hardwareMap.get(Servo.class, "grabber2");
        angle = hardwareMap.get(AnalogInput.class, "angle");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(DcMotor.Direction.FORWARD);
        arm2.setDirection(DcMotor.Direction.FORWARD);
        hanger.setDirection(DcMotor.Direction.FORWARD);
        winch.setDirection(DcMotor.Direction.FORWARD);
    
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
    
        waitForStart();
    
        // figure out where the arm is (what position index)
        double currentAngle = angle.getVoltage();
        double targetAngle = 0;
        double error = 10;
        for (int i=0; i<armPositions.length; i++) {
            if (Math.abs(armPositions[i]-currentAngle) < error) {
                error = Math.abs(armPositions[i]-currentAngle);
                armPosition = i;
            }
        }
        armPosition = 0;

        runtime.reset();
        double grabberPosition = 1.0;
        double grabberPosition2 = 0.1;
        long clickTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  -gamepad1.right_stick_x;
            
            boolean armUp = gamepad1.dpad_up;
            boolean armDown = gamepad1.dpad_down;
            
            boolean hangerDown = gamepad2.x;
            boolean hangerUp = gamepad2.y;
            boolean winchIn = gamepad2.a;
            boolean winchOut = gamepad2.b;
            
            double hangerPower = 0;
            if (hangerDown) {
                hangerPower = 1.0;
            } else if (hangerUp) {
                hangerPower = -1.0;
            }
            
            double winchPower;
            if (winchIn) {
                winchPower = 1.0;
                winchDefault = 0.3; // keep on pulling 5s after game over
            } else if (winchOut) {
                winchPower = -1.0;
                winchDefault = -0.3; // keep on pulling 5s after game over
            } else {
                winchPower = winchDefault;
            }
            
            double armPower = 0;
                
            // process clicks of the "arm position shifter" (dpad left right)
            if (clickTime == 0) {
                // inactive. process any clicks
                
                if (gamepad1.dpad_left) {
                    currentAngle = angle.getVoltage();
                    targetAngle = 0;
                    error = 10;
                    for (int i=0; i<armPositions.length; i++) {
                        if (Math.abs(armPositions[i]-currentAngle) < error) {
                            error = Math.abs(armPositions[i]-currentAngle);
                            armPosition = i;
                        }
                    }
                    armPosition = armPosition > 0 ? armPosition - 1:0;    
                } else if (gamepad1.dpad_right) {
                    currentAngle = angle.getVoltage();
                    targetAngle = 0;
                    error = 10;
                    for (int i=0; i<armPositions.length; i++) {
                        if (Math.abs(armPositions[i]-currentAngle) < error) {
                            error = Math.abs(armPositions[i]-currentAngle);
                            armPosition = i;
                        }
                    }
                    armPosition = armPosition < armPositions.length-1 ? armPosition + 1:armPosition;    
                }
                clickTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis()-clickTime > 500) {
                // active but enough time has passed. allow another click.
                clickTime = 0;
            }
                

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            
            // attenuate the power a bit for normal driving
            leftFrontPower  *= 0.3;
            rightFrontPower *= 0.3;
            leftBackPower   *= 0.3;
            rightBackPower  *= 0.3;
            
            if(armDown) {
                armPower = -0.2;
                armPosition = -1;
            } else if (armUp) {
                armPower = 0.2;
                armPosition = -1;
            } else {
                armPower = 0.0;
                armPosition = -1;
            }
            
            /*
            else if (armPosition != -1) {
                // work on getting the arm to the desired position
                // compare current angle to desired angle
                targetAngle = armPositions[armPosition];
                currentAngle = angle.getVoltage();
                
                
                if(Math.abs(targetAngle-currentAngle) > 0.01) {
                    // determine direction
                    double direction = -Math.signum(currentAngle-targetAngle);
                    // determine power based on distance
                    
                    if (Math.abs(targetAngle-currentAngle) > 0.2) {
                        armPower = 0.3*direction;
                        telemetry.addData("moving arm", "");
                    } else {
                        armPower = 0.2*direction;
                        telemetry.addData("moving arm", "");
                    }
                    arm1.setPower(armPower);
                    arm2.setPower(armPower);
                } else {
                    arm1.setPower(0.0);
                    arm2.setPower(0.0);
                }
            } else {
                 arm1.setPower(0.0);
                arm2.setPower(0.0);
            }
            */
            
            double slowFactor=0.30;
            if (gamepad1.left_trigger > 0.01){
                leftFrontPower*=slowFactor;
                rightFrontPower*=slowFactor;
                leftBackPower*=slowFactor;
                rightBackPower*=slowFactor;
                hangerPower *= slowFactor;
                armPower *=slowFactor;
            } else if (gamepad1.right_trigger>0.01){
                leftFrontPower*=(1/slowFactor)*gamepad1.right_trigger;
                rightFrontPower*=(1/slowFactor)*gamepad1.right_trigger;
                leftBackPower*=(1/slowFactor)*gamepad1.right_trigger;
                rightBackPower*=(1/slowFactor)*gamepad1.right_trigger;
                hangerPower /= slowFactor;
                armPower /=slowFactor;
            }
            if (gamepad2.left_trigger>0.01){
                hangerPower *= slowFactor;
                if (gamepad2.a || gamepad2.b) {
                    winchPower *= slowFactor;
                }
            } else if (gamepad2.right_trigger>0.01){
                hangerPower /= slowFactor;
                if (gamepad2.a || gamepad2.b) {
                    winchPower /= slowFactor;
                }
            }

            // Send calculated power to wheels and motors
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            arm1.setPower(armPower);
            arm2.setPower(armPower);
            hanger.setPower(hangerPower);
            winch.setPower(winchPower);
            
            if (gamepad1.a) {
                grabberPosition = 1.0;
                grabberPosition2 = 0.1;
                
            } else if (gamepad1.b) {
                grabberPosition = 0.1;
                grabberPosition2 = 1.0;
            }
            grabber.setPosition(grabberPosition);
            grabber2.setPosition(grabberPosition2);
            
            // wrist
            
            if (gamepad1.x) {
                wp = 0.03; // with arm angle 0.07
            } else if (gamepad1.y) {
                wp = 0.3;
            }
            wrist.setPosition(wp);
            grabber.setPosition(grabberPosition);
            grabber2.setPosition(grabberPosition2);
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm position index", "%1f", (double) armPosition);
            telemetry.addData("Current angle", "%4.2f", currentAngle);
            telemetry.addData("Target angle", "%4.2f", targetAngle);
            telemetry.addData("Arm power", "%4.2f", armPower);
            
            telemetry.addData("grabber", "%4.2f", grabberPosition);
            telemetry.addData("left y ", "%4.2f", gamepad1.left_stick_y);
            telemetry.addData("left x ", "%4.2f", gamepad1.left_stick_x);
            telemetry.addData("right x", "%4.2f", gamepad1.right_stick_x);
            telemetry.addData("wrist", "%4.2f", wp);
            telemetry.addData("winchPower", "%4.2f", winchPower);
            telemetry.update();
        }
    }}
