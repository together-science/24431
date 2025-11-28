package org.firstinspires.ftc.teamcode.util;

public class UtilThresholdButton {
    UtilLevelButtonInput levelButtonInput;
    double threshold;

    public UtilThresholdButton(UtilLevelButtonInput levelButtonInput, double threshold) {
        this.levelButtonInput = levelButtonInput;
        this.threshold = threshold;
    }

    public boolean getStatus() {
        return process(this.levelButtonInput.getInput());
    }

    private boolean process(double value) {
        return value > threshold;
    }
}
