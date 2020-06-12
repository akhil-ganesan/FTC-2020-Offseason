package org.firstinspires.ftc.teamcode.legacy.src;

public enum GameState {
    TeleOp(0, 120, "TeleOp"),
    EndGame(120, 150, "End Game");

    private final double startTime;
    private final double endTime;
    private final String name;

    GameState(double startTime, double endTime, String name) {
        this.startTime = startTime;
        this.endTime = endTime;
        this.name = name;
    }

    public double getStartTime() {
        return startTime;
    }

    public double getEndTime() {
        return endTime;
    }

    public String getName() {
        return name;
    }

}
