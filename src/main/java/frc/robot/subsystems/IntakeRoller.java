package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase{
    SparkFlex intakeroller = new SparkFlex(9,MotorType.kBrushless);
    public void spin() {
        intakeroller.set(0.2);
    }    
    public void spinBack() {
        intakeroller.set(-0.2);
    }   
    public void spinStop() {
        intakeroller.set(0);
    }
    public void spinToPoint(double point) {
        intakeroller.getClosedLoopController().setSetpoint(point,ControlType.kPosition);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeRollerPositionThingy",intakeroller.getEncoder().getPosition());
    }   
}


