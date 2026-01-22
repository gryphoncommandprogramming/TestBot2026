package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase{
    TalonFX climber = new TalonFX(11);
    public void up() {
        climber.set(0.2);
    }    
    public void down() {
        climber.set(-0.2);
    }   
    public void Stop() {
        climber.set(0);
    }
    public void spinToPoint(double point) {
        climber.setControl(new PositionVoltage(point));
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("climberPosition",climber.getPosition().getValueAsDouble());
    }    
}
