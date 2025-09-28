package frc.robot;

public class Helpers {
    public static enum HandlerState {
        STOWED,
        INTAKING,
        OUTTAKING,
        IDLE,
        MOVINGDOWN,
        MOVINGUP
    }

    public static int clamp(int val, int lower, int higher) {
        return Math.max(Math.min(val, higher), lower);
    }
}