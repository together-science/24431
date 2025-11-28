package org.firstinspires.ftc.teamcode.util;

public class Position {
    public double x, y, h;

    public Position(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public Position() {
    }

    static public double headingFromRelativePosition(double x, double y) {
        double h = Math.atan2(y, x)*(180/Math.PI)-90;
        return normalizeAngle(h);
    }

    static public double normalizeAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;

        return angle;
    }

}