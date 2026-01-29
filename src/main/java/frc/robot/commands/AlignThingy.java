package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GryphonLib.PositionCalculations;

public class AlignThingy extends Command {
    DriveSubsystem m_drive;
    public AlignThingy(DriveSubsystem m_drive) {
        this.m_drive = m_drive;
        
    }
    @Override
    public void initialize() {
        
    }
    public void execute(){
        //get angle between the angle i am pointing, and the angle i wish to be pointing
        //get there
        double desiredError = 0;
        double yawError = Math.atan(Math.tan(desiredError));
        SmartDashboard.putNumber("the yaw error", yawError);
        //turnPID calc
        
        //Clamp turn
        
        //Drive
        
        //Align
    
    }
}
