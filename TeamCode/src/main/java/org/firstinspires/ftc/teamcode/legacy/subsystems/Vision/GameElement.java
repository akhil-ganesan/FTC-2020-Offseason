package org.firstinspires.ftc.teamcode.legacy.subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.legacy.src.Constants;

public enum GameElement {
    ElementA("ElementA", Constants.targetsSkyStone.get(0), "Name1"),
    ElementB("ElementB", Constants.targetsSkyStone.get(1), "Name2");

    private final String TfodLabel;
    private final VuforiaTrackable vuforiaLabel;
    private final String name;

    GameElement(String TfodLabel, VuforiaTrackable VuforiaLabel, String name) {
        this.TfodLabel = TfodLabel;
        this.vuforiaLabel = VuforiaLabel;
        this.name = name;
    }

    public String getTfodLabel() {
        return TfodLabel;
    }

    public VuforiaTrackable getVuforiaLabel() {
        return vuforiaLabel;
    }

    public String getName() {
        return name;
    }

}
