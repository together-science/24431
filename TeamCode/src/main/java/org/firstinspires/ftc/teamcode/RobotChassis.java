package org.firstinspires.ftc.teamcode;

// an interface is a contract - classes that implement an interface
// can be used by code that uses the interface methods.
// this is kind of the answer to the missing multiple inheritance in Java

public interface RobotChassis {
    void stop(); // done
    void coast(); // done
    // will engage motors to drive in direction and turn toward it
    void startDrive(double speed, double direction, double turnSpeed);
    // will engage motors to strafe in direction and turn
    void startStrafe(double speed, double direction, double turnSpeed);
    // will engage motors to strafe in direction but with current heading
    void startStrafe(double speed, double direction);
    // engage motors to turn towards heading, on the spot
    void startTurn(double turnSpeed);

    double getHeading();

    double normalizeAngle(double angle);

    double headingFromRelativePosition(double x, double y);
    void turnTo(double turnSpeed, double heading);

    void turnToHeading(double maxTurnSpeed, double heading);

    void strafeDistance(final double maxDriveSpeed, final double distance, final double heading);


    // still to come: higher-level routines that iterate and wait (driveTo, strafeTo)
}
