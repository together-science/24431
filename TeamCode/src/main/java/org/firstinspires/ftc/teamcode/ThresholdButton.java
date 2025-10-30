package org.firstinspires.ftc.teamcode;

public class ThresholdButton {
    LevelButtonInput levelButtonInput;
    double threshold;

    ThresholdButton(LevelButtonInput levelButtonInput, double threshold) {
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
