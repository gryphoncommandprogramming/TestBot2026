package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MoveIntake extends SubsystemBase{
    SparkFlex moveIntake = new SparkFlex(10,MotorType.kBrushless);
    public void rollOut() {
        moveIntake.set(0.1);
    }    
    public void rollBack() {
        moveIntake.set(-0.1);
    }   
    public void rollStop() {
        moveIntake.set(0);
    }   
    public void rollToIntake() {
        moveIntake.getClosedLoopController().setSetpoint(0.25,ControlType.kPosition);
    }
    public void rollToRest() {
        moveIntake.getClosedLoopController().setSetpoint(0.0,ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeMoverPosition",moveIntake.getEncoder().getPosition());
    }
}
