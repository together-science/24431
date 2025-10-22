/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class ScorpColor extends LinearOpMode {

    private NormalizedColorSensor colorSensor;

    ScorpColor

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color"); // Replace "your_color_sensor_name" with the actual name configured in the robot controller

        // You can also configure the sensor for specific settings if needed
        // colorSensor.setGain(2.0); // Example: set gain

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read normalized colors
            float red = colorSensor.normalizedColors.red;
            float green = colorSensor.normalizedColors.green;
            float blue = colorSensor.normalizedColors.blue;
            float alpha = colorSensor.normalizedColors.alpha; // Proximity value

            // You can also get the color as an ARGB integer
            int color = colorSensor.normalizedColors.toColor();

            // Display color values on the telemetry
            telemetry.addData("Red", "%.3f", red);
            telemetry.addData("Green", "%.3f", green);
            telemetry.addData("Blue", "%.3f", blue);
            telemetry.addData("Alpha (Proximity)", "%.3f", alpha);
            telemetry.addData("ARGB Color", "0x%08X", color);

            // You can use these values for your robot's logic, e.g., identifying colors
            // if (red > green && red > blue) {
            //     telemetry.addData("Detected Color", "Red");
            // } else if (green > red && green > blue) {
            //     telemetry.addData("Detected Color", "Green");
            // } else if (blue > red && blue > green) {
            //     telemetry.addData("Detected Color", "Blue");
            // }

            telemetry.update();
        }
    }

}
*/