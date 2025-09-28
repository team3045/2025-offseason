package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public class EndEffectorConstants {
    public static int EFFECTORTILTERID = 21;
    public static int EFFECTORID = 30;
    public static int ENCODERBOTTOMID = 31;
    public static int ENCODERTOPID = 32;
    public static double ENCODERROTDIFFPERFULLROT = 0.01;
    public static double SPEEDDOWN = 1;
    public static double HOLDSPEED = 0.01;
    public static double ANGLETOLERANCE = 0.01;
    public static double EFFECTORSPEED = 0.5;
    public static double EFFECTORHOLDSPEED = -0.01;
    public static int CORALRANGEID = 37;
    public static int ALGAERANGEID = 37;
    public static double DEFAULTCORALDIST = 0.1;
    public static double DEFAULTALGAEDIST = 0.5;
    public static double STOWANGLE = Units.degreesToRotations(0);
    public static double CORALINTAKEANGLE = Units.degreesToRotations(100);
    public static double ALGAEINTAKEANGLE = Units.degreesToRotations(-100);
}
