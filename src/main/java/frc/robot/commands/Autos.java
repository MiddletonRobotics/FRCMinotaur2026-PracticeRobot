package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Autos {
    private final RobotContainer robotContainer;
    private final Drivetrain drivetrain;
    private final AutoFactory autoFactory;

    public Autos() {
        robotContainer = new RobotContainer();
        drivetrain = robotContainer.getDrivetrain();

        autoFactory = new AutoFactory(
            drivetrain::getPose, 
            drivetrain::setPose, 
            drivetrain::followTrajectory, 
            true, 
            drivetrain
        );
    }

    public AutoRoutine leftDriveAndScore() {
        AutoRoutine routine = autoFactory.newRoutine("LDriveThenScoreThenHang");

        AutoTrajectory driveToShoot = routine.trajectory("driveToShoot");

        routine.active().onTrue(
            Commands.sequence(
                driveToShoot.resetOdometry(),
                driveToShoot.cmd()
            )
        );

        return routine;
    } 
}