package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CustomAlerts;



public class Climber extends SubsystemBase {
    //subsystem components such as motors:
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    ClimberIO io;
    
    //Constructor
    public Climber(ClimberIO iO) {
        io = iO;
        CustomAlerts.makeOverTempAlert(() -> inputs.temperatures[0], 60, 50,"Left Climber motor");
        CustomAlerts.makeOverTempAlert(() -> inputs.temperatures[1], 60, 50,"Right Climber motor");
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
    
}