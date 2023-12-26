package frc.robot;

public class Constants {

    public static final class IntakeConst{
        // change constants later

        public static final int motorPort = 45;

        public static final double idleSpeed = 0.1;

        public static final double intakeconeSpeed = 0.4;
        public static final double intakecubeSpeed = -0.4; // one direction is for cone, other is for cube
        
        public static final double outtakeconeSpeed = -0.9;
        public static final double outtakecubeSpeed = 0.9;

        public static final double currentThreshold = 60.0;
    }

    public static final class controller{
        public static final int controllerPort = 0;
        // public static final int LT = 9;
        // public static final int RT = 10;
        public static final int LT = 5;
        public static final int RT = 6;
    }
}