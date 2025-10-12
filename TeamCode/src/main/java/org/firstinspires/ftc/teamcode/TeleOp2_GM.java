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


@TeleOp(name="TeleOp 2-GM", group="Linear Opmode")

public class TeleOp2_GM extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    //private DcMotor arm1 = null;
    //private DcMotor arm2 = null;
    //private DcMotor hanger = null;
    //private Servo grabber = null;
    //private AnalogInput angle = null;
    private IMU imu = null;

    
    static final double armPositions[] = new double[]{
        3.27, // init
        0.99, // basket
        0.75, // chamber2
        0.64, // under
        0.53, // out
        0.05, // pickup
        };
       
    static final int ARM_INIT = 0;
    static final int ARM_BASKET = 1;
    static final int ARM_CHAMBER2= 2;
    static final int ARM_UNDER = 3;
    static final int ARM_OUT = 4;
    static final int ARM_PICKUP = 5;
    
    private int armPosition = ARM_INIT;
    private boolean globalMode = false;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        //arm1 = hardwareMap.get(DcMotor.class, "arm1");
        //arm2 = hardwareMap.get(DcMotor.class, "arm2");
        //hanger = hardwareMap.get(DcMotor.class, "hanger");
        //grabber = hardwareMap.get(Servo.class, "grabber");
        //angle = hardwareMap.get(AnalogInput.class, "angle");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //arm1.setDirection(DcMotor.Direction.FORWARD);
        //arm2.setDirection(DcMotor.Direction.FORWARD);
        //hanger.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
    
        waitForStart();
    
        // figure out where the arm is (what position index)
        double currentAngle = 0; //angle.getVoltage();
        double targetAngle = 0;
        double error = 10;
        for (int i=0; i<armPositions.length; i++) {
            if (Math.abs(armPositions[i]-currentAngle) < error) {
                error = Math.abs(armPositions[i]-currentAngle);
                armPosition = i;
            }
        }

        runtime.reset();
        double grabberPosition = 0.1;
        long clickTime = 0;
        double currentHeading;
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Float Mode uses left joystick to go forward & strafe,
            // and right joystick to rotate.
            
            double x = gamepad1.left_stick_x;  
            double y = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double yaw = gamepad1.right_stick_x;
            
            double desiredHeading = Math.atan2(y,x)/Math.PI*180;
            currentHeading = getHeading();
            double delta = desiredHeading - currentHeading;
            // Normalize the delta to be within +/- 180 degrees
            while (delta >= 180)  delta -= 360;
            while (delta <= -180) delta += 360;
            
            double lateral = Math.cos(delta/180*Math.PI);
            double axial = Math.sin(delta/180*Math.PI);
            double power = Math.sqrt(x*x+y*y);
            
            if (power>=0.01) {
                final double POWER_MULT = 3;
                power *= POWER_MULT;
                axial *= power;
                lateral *= power;
                yaw *= power;
            } else {
                power = 0;
                axial = 0;
                lateral = 0;
            }
            
            if (!globalMode) {
                axial   = y;
                lateral = x;
            }
            
            boolean armUp = gamepad1.dpad_up;
            boolean armDown = gamepad1.dpad_down;
            
            boolean hangerDown = gamepad2.x;
            boolean hangerUp = gamepad2.y;
            
            double hangerPower = 0;
            if (hangerDown) {
                hangerPower = 1.0;
            } else if (hangerUp) {
                hangerPower = -1.0;
            }
            
            double armPower = 0;
                
            // process clicks of the "arm position shifter" (dpad left right)
            if (clickTime == 0) {
                // inactive. process any clicks
                
                if (gamepad1.dpad_left) {
                    currentAngle = 0; //angle.getVoltage();
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
                    currentAngle = 0; //angle.getVoltage();
                    targetAngle = 0;
                    error = 10;
                    for (int i=0; i<armPositions.length; i++) {
                        if (Math.abs(armPositions[i]-currentAngle) < error) {
                            error = Math.abs(armPositions[i]-currentAngle);
                            armPosition = i;
                        }
                    }
                    armPosition = armPosition < armPositions.length-1 ? armPosition + 1:armPosition;    
                } else if (gamepad1.right_bumper) {
                    globalMode = !globalMode;
                    if (globalMode) {
                        imu.resetYaw();
                    }
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
            
            if(armUp) {
                armPower = -0.2;
                armPosition = -1;
                //arm1.setPower(armPower);
                //arm2.setPower(armPower);
            } else if (armDown) {
                armPower = 0.2;
                armPosition = -1;
                //arm1.setPower(armPower);
                //arm2.setPower(armPower);
            } else if (armPosition != -1) {
                // work on getting the arm to the desired position
                // compare current angle to desired angle
                targetAngle = armPositions[armPosition];
                currentAngle = 0; //angle.getVoltage();
                
                
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
                    //arm1.setPower(armPower);
                    //arm2.setPower(armPower);
                } else {
                    //arm1.setPower(0.0);
                    //arm2.setPower(0.0);
                }
            } else {
                //arm1.setPower(0.0);
                //arm2.setPower(0.0);
            }
            
            double slowFactor=0.30;
            if (gamepad1.left_bumper){
                // a little faster
                slowFactor = 0.6;
            } else if (gamepad1.left_trigger>0.01){
                // full power
                slowFactor = 1.0;
            }
            leftFrontPower*=slowFactor;
            rightFrontPower*=slowFactor;
            leftBackPower*=slowFactor;
            rightBackPower*=slowFactor;
           
            if (gamepad2.left_trigger<=0.01){
                hangerPower *= slowFactor;
            }

            // Send calculated power to wheels and motors
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //arm1.setPower(armPower);
            //arm2.setPower(armPower);
            //hanger.setPower(hangerPower);
            
            if (gamepad1.a) {
                grabberPosition = 1.0;
                
            } else if (gamepad1.b) {
                grabberPosition = 0.1;
            }
            //grabber.setPosition(grabberPosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm position index", "%1f", (double) armPosition);
            telemetry.addData("Current angle", "%4.2f", currentAngle);
            telemetry.addData("Target angle", "%4.2f", targetAngle);
            telemetry.addData("Arm power", "%4.2f", armPower);
            
            telemetry.addData("grabber", "%4.2f", grabberPosition);
            telemetry.addData("y ", "%4.2f", y);
            telemetry.addData("x ", "%4.2f", x);
            telemetry.addData("turn", "%4.2f", gamepad1.right_stick_x);
            telemetry.addData("ch", "%4.2f", currentHeading);
            telemetry.addData("dh", "%4.2f", desiredHeading);
            telemetry.addData("delta", "%4.2f", delta);
            telemetry.addData("power", "%4.2f", power);
            telemetry.addData("lateral", "%4.2f", lateral);
            telemetry.addData("axial", "%4.2f", axial);
            telemetry.addData("mode: ", globalMode?"global":"pov");
            telemetry.update();
        }
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
