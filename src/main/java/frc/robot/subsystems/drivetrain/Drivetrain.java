package frc.robot.subsystems.drivetrain;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Drivetrain {
    public static Lock odometryLock = new ReentrantLock();
}
