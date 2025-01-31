package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climber extends SubsystemBase {
    //subsystem components such as motors:
    
    private final SparkMax climberLeft = new SparkMax( 1, MotorType.kBrushless ); // temporary ID
    private final SparkMax climberRight = new SparkMax( 1, MotorType.kBrushless ); // temporary ID

    //Constructor
    public Climber() {}

    //Custom methods go here:
    public void setPosition(double position) {
        climberLeft.getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl);
        climberRight.getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl);

    }
}