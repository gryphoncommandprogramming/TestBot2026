package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.MoveIntake;

public class RollerSpin extends Command {
    IntakeRoller intakeRoller;
    MoveIntake moveIntake;
    public RollerSpin(IntakeRoller intakeRoller, MoveIntake moveIntake) {
        this.intakeRoller = intakeRoller;
        this.moveIntake = moveIntake;
        addRequirements(intakeRoller, moveIntake);
    }
    @Override
    public void initialize() {
        moveIntake.rollToIntake();
    }
    @Override
    public void execute() {
        intakeRoller.setVelocity(0);
    }
    @Override
    public void end(boolean interrupted) {
        intakeRoller.setVelocity(0);
        moveIntake.rollToRest();
    }
}
