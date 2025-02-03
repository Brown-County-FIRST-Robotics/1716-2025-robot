package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs{
        double positions[] = new double[]{0.0,0.0};
        double velocities[] = new double[]{0.0,0.0};
        double temperatures[] = new double[]{0.0,0.0};
        double currents[] = new double[]{0.0,0.0};
        double appliedOutputs[] = new double[]{0.0,0.0};

    }
    default void updateInputs(ClimberIOInputs inputs){}

    default void setVelocities(double velocityLeft, double velocityRight){}    
}
