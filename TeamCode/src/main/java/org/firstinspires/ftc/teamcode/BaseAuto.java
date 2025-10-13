package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.io.IOException;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class BaseAuto extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         rightFrontDrive  = null;
    private DcMotor         leftBackDrive   = null;
    private DcMotor         rightBackDrive  = null;
    private IMU             imu         = null; 
    public SparkFunOTOS     myOtos;
    private double          headingError  = 0;

    private ScorpCannon leftCannon = null;
    private ScorpCannon rightCannon = null;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    protected double  targetHeading = 0;
    protected double  targetX = 0;
    protected double  targetY = 0;
    private double    driveSpeed    = 0;
    private double    turnSpeed     = 0;
    private double    leftSpeed     = 0;
    private double    rightSpeed    = 0;
    private int       leftFrontTarget    = 0;
    private int       rightFrontTarget   = 0;
    private int       leftBackTarget    = 0;
    private int       rightBackTarget   = 0;
    
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7*(24.0/35.0) ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     STRAFE_CORRECTION       = 1.0; // multiplier for lateral portion of strafe, see strafe()

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED_FAST        = 0.4;     // Max driving speed for better distance accuracy.
    static final double     DRIVE_SPEED_NORMAL      = 0.2;     // Max driving speed for better distance accuracy.
    static final double     DRIVE_SPEED_SLOW        = 0.1;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.5;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    protected void autoInit() {
    }

    protected void auto() {
    }
    
    
    protected void testAxes(){
        while(opModeIsActive()){
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        telemetry.addData("Heading", pos.h);
        telemetry.addData("PosX", pos.x);
        telemetry.addData("PosY", pos.y);
        // telemetry.update
        }
    }
    
    /*
    //double posX;
    //double posY;
    //double distance = 0;
    //double dx;
    //double dy;
    //double h;
    */ //This is moveTo() functions variables defined globally if needed for telemetry
    
    protected void moveTo(double x, double y, double driveSpeed, double accuracy){
        SparkFunOTOS.Pose2D pos;
        double posX;
        double posY;
        double distance = 1000;
        double dx;
        double dy;
        double h;
        while(Math.abs(distance) > accuracy && opModeIsActive()) {
            pos = myOtos.getPosition();
            posX = pos.x;
            posY = pos.y;
            posX*=-1; 
            posY*=-1;
            dx = x - posX;
            dy = y - posY;
            distance = Math.sqrt(dx*dx+dy*dy);
            h = (Math.atan2(dy, dx)*(180/3.1415)-90);
            h = h < 0 ? h+360 : h;
            startDriveStraight(driveSpeed, h);
        }
        moveRobot(0, 0);
    }
    
    public void setUpOtos(){
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0); //This needs updating
        myOtos.setAngularScalar(1.0); // So does this

        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
    
    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "oscar");
        leftCannon = new ScorpCannon(hardwareMap, "left_cannon_wheel", "left_cannon_trigger");
        rightCannon = new ScorpCannon(hardwareMap, "right_cannon_wheel", "right_cannon_trigger");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        autoInit();
        
        setUpOtos();

        while (opModeInInit()) {
            telemetry.addData(">", "Waiting ...");
            telemetry.update();
        }

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        auto();
        
        telemetry.update();
        telemetry.addData("Path", "Complete");
        sleep(1000);
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
    *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the OpMode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    
    public void driveStraight(double maxDriveSpeed, double distance) {driveStraight(maxDriveSpeed, distance, getHeading());}
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            leftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            moveRobot(Math.abs(maxDriveSpeed), 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    
    }
    public void startDriveStraight(double maxDriveSpeed, double h) {
            turnSpeed = getSteeringCorrection(h, P_DRIVE_GAIN);
            moveRobot(Math.abs(maxDriveSpeed), turnSpeed);
    }
    public void strafe(double maxDriveSpeed, double distance, double heading) {
        if (opModeIsActive()) {
            
            double axial   = Math.cos(Math.PI/180*heading);  // Note: pushing stick forward gives negative value
            double lateral =  -Math.sin(Math.PI/180*heading);
            
            telemetry.addData("axial  ", "%5.2f", axial);
            telemetry.addData("lateral", "%5.2f", lateral);
            telemetry.update();
            
            double leftFrontPower  = (axial + lateral);
            double rightFrontPower = (axial - lateral);
            double leftBackPower   = (axial - lateral);
            double rightBackPower  = (axial + lateral);
            
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            
            leftFrontPower *= maxDriveSpeed;
            rightFrontPower *= maxDriveSpeed;
            leftBackPower *= maxDriveSpeed;
            rightBackPower *= maxDriveSpeed;

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH * (1+STRAFE_CORRECTION*Math.abs(lateral)));
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)Math.signum(leftFrontPower)*moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() +  (int)Math.signum(rightFrontPower)*moveCounts;
            leftBackTarget = leftBackDrive.getCurrentPosition() +  (int)Math.signum(leftBackPower)*moveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() +  (int)Math.signum(rightBackPower)*moveCounts;
            
            

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            double robotHeading = getHeading();

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(robotHeading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(false);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {
        //sendTelemetry(true);
        targetHeading = heading;
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFrontDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        leftBackDrive.setPower(leftSpeed);
        rightBackDrive.setPower(rightSpeed);
        
    }

    public void strafeRobot(double lf, double rf, double lb, double rb, double turn) {
        driveSpeed = lf;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        lf -= turn;
        rf += turn;
        lb -= turn;
        rb += turn;
        
        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(lf), Math.abs(rf));
        if (max > 1.0)
        {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
        
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    protected void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftFrontTarget,  rightFrontTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    /**
     * The -pos.y and -pos.x returns could account for the mounting direction
     *    of the sensor on the robot - but let's handle it in moveTo routine instead
     */
    public SparkFunOTOS.Pose2D getPosition() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        
        //return new SparkFunOTOS.Pose2D(-pos.y, -pos.x, pos.h);
        return new SparkFunOTOS.Pose2D(pos.y, pos.x, pos.h);
    }

   
/** A simple way to log data to a file */

public class Log {
    private static final String BASE_FOLDER_NAME = "FIRST";
    private Writer fileWriter;
    private String line;
    private boolean logTime;
    private long startTime;
    private boolean disabled = false;
    //private String fileName ="data";

       
    Log(String fileName,boolean logTime){
    //   if (logTime) startTime = System.nanoTime();
    //   this.logTime = logTime;
       //String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
       String directoryPath = "/sdcard/FIRST/" + BASE_FOLDER_NAME;
       File directory = new File(directoryPath);
       //no inspection ResultOfMethodCallIgnored
      
       if (!directory.isDirectory())
       {
           directory.mkdir();
           //if(!fDir.mkdir()){
           //    DbgLog.error("Could not make directory + directoryPath");
           //}
       }
       
       try{
           fileWriter = new FileWriter(directoryPath+"/"+fileName+".txt");
       } catch (IOException e){
           e.printStackTrace();
       }
    }   
       public boolean isDisable(){
           return disabled;
       }
       
       public void setDisabled(boolean disable){
           this.disabled = disabled;
       }
       
       public void close(){
           try{
               fileWriter.close();
           } catch (IOException e){
               e.printStackTrace();
           }
           
       }
    
    public void update() {
        if (disabled) return;
        try {
              line = "TEST sentence";
              fileWriter.write(line+"\n");
              line = "";
            } catch (IOException e) {
                e.printStackTrace();
        }
        
    }
    
    public void addData(String data) {
        if (disabled) return;
        if (!line.equals("")) line += ",";
        line += data;
    }
    
    public void addData(Object data){
       addData(data.toString()); 
    }
    public void addData(boolean data){
       addData(String.valueOf(data)); 
    }
        public void addData(byte data){
       addData(String.valueOf(data)); 
    }
        public void addData(char data){
       addData(String.valueOf(data)); 
    }
        public void addData(short data){
       addData(String.valueOf(data)); 
    }
        public void addData(int data){
       addData(String.valueOf(data)); 
    }
        public void addData(long data){
       addData(String.valueOf(data)); 
    }
        public void addData(float data){
       addData(String.valueOf(data)); 
    }
        public void addData(double data){
       addData(String.valueOf(data)); 
    }
   
    
    // todo: write your code here
}
    
}
