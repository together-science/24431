package org.firstinspires.ftc.teamcode;

public class UtilThresholdButton {
    UtilLevelButtonInput levelButtonInput;
    double threshold;

    UtilThresholdButton(UtilLevelButtonInput levelButtonInput, double threshold) {
        this.levelButtonInput = levelButtonInput;
        this.threshold = threshold;
    }

    boolean getStatus() {
        return process(this.levelButtonInput.getInput());
    }

    private boolean process(double value) {
        return value > threshold;
    }
}
