package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class GlobalConstants {
    public static final Mode kSimulationMode = Mode.SIM;
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimulationMode;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
