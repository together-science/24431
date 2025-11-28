package org.firstinspires.ftc.teamcode.util;

public class ThresholdButton {
    LevelButtonInput levelButtonInput;
    double threshold;

    public ThresholdButton(LevelButtonInput levelButtonInput, double threshold) {
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
