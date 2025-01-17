package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private final SparkMax elevator = new SparkMax(0, MotorType.kBrushless); //TEMP IDs
    private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    private final SparkFlex wrist = new SparkFlex(1, MotorType.kBrushless);
    private final SparkFlexConfig wristConfig = new SparkFlexConfig();
    private final SparkFlex gripper = new SparkFlex(2, MotorType.kBrushless);
    private final SparkFlexConfig gripperConfig = new SparkFlexConfig();

    public enum ManipulatorPosition {
        INTAKE, PROCESSOR, TROUGH, LEVEL2, LEVEL3, LEVEL4, ALGAELOW, ALGAEHIGH
    }

    public Manipulator() {
        //TODO: configure motor closed loops
        elevatorConfig.closedLoop.maxMotion.maxAcceleration(1200); //placeholder
        wristConfig.closedLoop.maxMotion.maxAcceleration(1200); //placeholder
        gripperConfig.closedLoop.maxMotion.maxAcceleration(12000);

        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        gripper.configure(gripperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void SetPosition(ManipulatorPosition position) {
        switch (position) { //TODO: Replace placeholder position values
            case INTAKE:
                elevator.getClosedLoopController().setReference(1.0, ControlType.kMAXMotionPositionControl);
                wrist.getClosedLoopController().setReference(1.0, ControlType.kMAXMotionPositionControl);
                break;
            case PROCESSOR:
                elevator.getClosedLoopController().setReference(2.0, ControlType.kMAXMotionPositionControl);
                wrist.getClosedLoopController().setReference(2.0, ControlType.kMAXMotionPositionControl);
                break;
            case TROUGH:
                elevator.getClosedLoopController().setReference(3.0, ControlType.kMAXMotionPositionControl);
                wrist.getClosedLoopController().setReference(3.0, ControlType.kMAXMotionPositionControl);
                break;
            case LEVEL2:
                elevator.getClosedLoopController().setReference(4.0, ControlType.kMAXMotionPositionControl);
                wrist.getClosedLoopController().setReference(4.0, ControlType.kMAXMotionPositionControl);
                break;
            case LEVEL3:
                elevator.getClosedLoopController().setReference(5.0, ControlType.kMAXMotionPositionControl);
                wrist.getClosedLoopController().setReference(5.0, ControlType.kMAXMotionPositionControl);
                break;
            case LEVEL4:
                elevator.getClosedLoopController().setReference(6.0, ControlType.kMAXMotionPositionControl);
                wrist.getClosedLoopController().setReference(6.0, ControlType.kMAXMotionPositionControl);
                break;
            case ALGAELOW:
                elevator.getClosedLoopController().setReference(7.0, ControlType.kMAXMotionPositionControl);
                wrist.getClosedLoopController().setReference(7.0, ControlType.kMAXMotionPositionControl);
                break;
            case ALGAEHIGH:
                elevator.getClosedLoopController().setReference(8.0, ControlType.kMAXMotionPositionControl);
                wrist.getClosedLoopController().setReference(8.0, ControlType.kMAXMotionPositionControl);
                break;
        }
    }

    public void Intake() {
        gripper.getClosedLoopController().setReference(-0.5, ControlType.kMAXMotionVelocityControl);
    }

    public void Deposit() {
        gripper.getClosedLoopController().setReference(0.5, ControlType.kMAXMotionVelocityControl);
    }
}
