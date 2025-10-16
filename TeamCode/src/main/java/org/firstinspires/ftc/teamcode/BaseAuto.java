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
    private ScorpCannon leftCannon = null;
    private ScorpCannon rightCannon = null;
    private ScorpChassis chassis = null;
    private ScorpIntake intake = null;
    private ScorpSorter sorter = null;

    private double    headingError  = 0;
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

    @Override
    public void runOpMode() {
        leftCannon = new ScorpCannon(hardwareMap, "left_cannon_wheel", "left_cannon_trigger");
        rightCannon = new ScorpCannon(hardwareMap, "right_cannon_wheel", "right_cannon_trigger");
        chassis = new ScorpChassis(this, hardwareMap, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        intake = new ScorpIntake(hardwareMap, "left_intake", "right_intake");
        sorter = new ScorpSorter(hardwareMap, "sorter_servo");

        chassis.init();

        autoInit();

        while (opModeInInit()) {
            telemetry.addData(">", "Waiting ...");
            telemetry.update();
        }

        auto();
    } //Perfect
