// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.GryphonLib.PositionCalculations;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PositionPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.MoveIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.MoveIntake;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // Subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final IntakeRoller intakeRoller = new IntakeRoller();
  private final MoveIntake deployIntake = new MoveIntake();
  private final Climber climber = new Climber();
  private final Hood hood = new Hood();
  private final FlyWheel flywheelroller = new FlyWheel();

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  public RobotContainer() {
    new Vision();
    configureDefaultCommands();
    configureButtonBindings();
    configureStateTriggers();
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(
        new RunCommand(
            () -> {
              double forward = m_driverController.getLeftY();
              double strafe = m_driverController.getLeftX();
              double turn = m_driverController.getRightX();

              m_drive.drive(
                -MathUtil.applyDeadband(forward, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(strafe, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(turn, OIConstants.kDriveDeadband), true);
            },
            m_drive));
  }

  private void configureButtonBindings() {
    SmartDashboard.putNumber("RPM",flywheelroller.flywheelroller.getEncoder().getVelocity());
    // Driver bindings
    m_driverController.start().onTrue(new InstantCommand(()->m_drive.zeroHeading(), m_drive));
    m_driverController.leftBumper().whileTrue(Commands.defer(()->PositionPIDCommand.generateCommand(m_drive, PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, 2), Seconds.of(2)), Set.of(m_drive)));
    m_driverController.rightBumper().whileTrue(Commands.defer(()->m_drive.goToPose(PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, -2)), Set.of(m_drive)));
    m_driverController.a().whileTrue(Commands.defer(()->PositionPIDCommand.generateCommand(m_drive, PositionCalculations.getStraightOutPose(8), Seconds.of(2)), Set.of(m_drive)));
    m_driverController.b().whileTrue(Commands.defer(()->m_drive.goToPose(PositionCalculations.getStraightOutPose(8)), Set.of(m_drive)));
    m_driverController.x().whileTrue(new InstantCommand(()->intakeRoller.spin(),intakeRoller));
    m_driverController.x().onFalse(new InstantCommand(()->intakeRoller.spinStop(),intakeRoller));
    m_driverController.y().whileTrue(new InstantCommand(()->intakeRoller.spinBack(),intakeRoller));
    m_driverController.y().onFalse(new InstantCommand(()->intakeRoller.spinStop(),intakeRoller));
    m_driverController.leftTrigger().whileTrue(new InstantCommand(()->deployIntake.rollOut(),deployIntake));
    m_driverController.leftTrigger().onFalse(new InstantCommand(()->deployIntake.rollStop(),deployIntake));
    m_driverController.rightTrigger().whileTrue(new InstantCommand(()->deployIntake.rollOut(),deployIntake));
    m_driverController.rightTrigger().onFalse(new InstantCommand(()->deployIntake.rollStop(),deployIntake));
    m_driverController.povDown().whileTrue(new InstantCommand(()->climber.down(),climber));
    m_driverController.povUp().whileTrue(new InstantCommand(()->climber.up(),climber));
    m_driverController.povUp().onFalse(new InstantCommand(()->climber.Stop(),climber));
    m_driverController.povDown().onFalse(new InstantCommand(()->climber.Stop(),climber));
    m_driverController.povRight().whileTrue(new InstantCommand(()->hood.spin(),hood));
    m_driverController.povRight().onFalse(new InstantCommand(()->hood.spinStop(),hood));
    m_driverController.povLeft().whileTrue(new InstantCommand(()->hood.spinBack(),hood));
    m_driverController.povLeft().onFalse(new InstantCommand(()->hood.spinStop(),hood));

    m_operatorController.leftBumper().whileTrue(new InstantCommand(()->flywheelroller.setVelocity(SmartDashboard.getNumber("RPM", 0))));

    SmartDashboard.putData("Drive 2m Back", Commands.defer(()->m_drive.goToPose(PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, -2)), Set.of(m_drive)));
    SmartDashboard.putData("Drive 2m Forward", new InstantCommand(()->m_drive.goToPose(PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, 2)).schedule()));
    SmartDashboard.putData("PID 2m Forward", new InstantCommand(()->PositionPIDCommand.generateCommand(m_drive, PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, 2), Seconds.of(2)).schedule()));
  
  }

  private void configureStateTriggers() {}
    

  /** Returns the autonomous command. */
  public Command getAutonomousCommand() {
    // return m_drive.PathToPose(new Pose2d(new Translation2d(4, 6), new Rotation2d(3*Math.PI/4)), 0).andThen(
    //   m_drive.PathToPose(new Pose2d(new Translation2d(12, 2), new Rotation2d(11*Math.PI/6)), 0)
    // );

    return m_drive.goToPose(new Pose2d(new Translation2d(4, 6), new Rotation2d(3*Math.PI/4))).andThen(
      m_drive.goToPose(new Pose2d(new Translation2d(12, 2), new Rotation2d(11*Math.PI/6)))
    );
  
  }
}
