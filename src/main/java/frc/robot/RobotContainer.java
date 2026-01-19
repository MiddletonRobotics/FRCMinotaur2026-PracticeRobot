// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DrivetrainCommands;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.Mode;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOHardware;
import frc.robot.subsystems.drivetrain.GyroIOSimulation;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOHardware;
import frc.robot.subsystems.drivetrain.ModuleIOSimulation;

public class RobotContainer {
  private final Drivetrain drivetrain;
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final LoggedDashboardChooser<Command> autonomousChooser;

  private SwerveDriveSimulation driveSimulation = null;

  public RobotContainer() {
    switch (GlobalConstants.kCurrentMode) {
      case REAL:
        drivetrain = new Drivetrain(
          new GyroIOHardware(), 
          new ModuleIOHardware(0, DrivetrainConstants.kFrontLeftModuleConstants), 
          new ModuleIOHardware(1, DrivetrainConstants.kFrontRightModuleConstants), 
          new ModuleIOHardware(2, DrivetrainConstants.kBackLeftModuleConstants),
          new ModuleIOHardware(3, DrivetrainConstants.kBackRightModuleConstants),
          (pose) -> {}
        );

        break;
      case SIM:
        this.driveSimulation = new SwerveDriveSimulation(DrivetrainConstants.kMapleSimConfiguration, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drivetrain = new Drivetrain(
          new GyroIOSimulation(driveSimulation.getGyroSimulation()), 
          new ModuleIOSimulation(driveSimulation.getModules()[0]), 
          new ModuleIOSimulation(driveSimulation.getModules()[1]), 
          new ModuleIOSimulation(driveSimulation.getModules()[2]), 
          new ModuleIOSimulation(driveSimulation.getModules()[3]),
          driveSimulation::setSimulationWorldPose
        );

        break;
      default:
        drivetrain = new Drivetrain(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
        break;
    }

    autonomousChooser = new LoggedDashboardChooser<>("Auton Choices", AutoBuilder.buildAutoChooser());
    autonomousChooser.addOption("Drivetrain Wheel Radius Characterization", DrivetrainCommands.wheelRadiusCharacterization(drivetrain));
    autonomousChooser.addOption("Drivetrain Simple FF Characterization", DrivetrainCommands.feedforwardCharacterization(drivetrain));
    autonomousChooser.addOption("Drivetrain SysId (Quasistatic Forward)", drivetrain.sysIdQuasistatic(Direction.kForward));
    autonomousChooser.addOption("Drivetrain SysId (Quasistatic Reverse)", drivetrain.sysIdQuasistatic(Direction.kReverse));
    autonomousChooser.addOption("Drivetrain SysId (Dynamic Forward)", drivetrain.sysIdDynamic(Direction.kForward));
    autonomousChooser.addOption("Drivetrain SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(Direction.kReverse));

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(DrivetrainCommands.joystickDrive(
      drivetrain, 
      () -> -primaryController.getLeftY(), 
      () -> -primaryController.getLeftX(), 
      () -> -primaryController.getRightX()
    ));

    primaryController.a().whileTrue(DrivetrainCommands.joystickDriveAtAngle(
      drivetrain,
      () -> -primaryController.getLeftY(),
      () -> -primaryController.getLeftX(),
      () -> Rotation2d.kZero
    ));

    primaryController.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
    primaryController.b().onTrue(Commands.runOnce(() ->
      drivetrain.setPose(new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.kZero)), drivetrain).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.get();
  }

  public void resetSimulationField() {
    if (GlobalConstants.kCurrentMode != Mode.SIM) return;

    drivetrain.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (GlobalConstants.kCurrentMode != Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}