package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{
    SparkFlex hood = new SparkFlex(12,MotorType.kBrushless);
    public void spin() {
        hood.set(0.2);
    }    
    public void spinBack() {
        hood.set(-0.2);
    }   
    public void spinStop() {
        hood.set(0);
    }
    public void spinToPoint(double point) {
        hood.getClosedLoopController().setSetpoint(point,ControlType.kPosition);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("HoodPositionThingy",hood.getEncoder().getPosition());
    }   
}

