package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlyWheel extends SubsystemBase{
    public SparkFlex flywheelroller = new SparkFlex(9,MotorType.kBrushless);
    public void spin() {
        flywheelroller.set(0.2);
    }    
    public void spinBack() {
        flywheelroller.set(-0.2);
    }   
    public void spinStop() {
        flywheelroller.set(0);
    }
    public void setVelocity(double velocity) {
        flywheelroller.getClosedLoopController().setSetpoint(velocity,ControlType.kVelocity);

    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("FlyWheelRollerVelocityThingy",flywheelroller.getEncoder().getVelocity());
    }

    
}


