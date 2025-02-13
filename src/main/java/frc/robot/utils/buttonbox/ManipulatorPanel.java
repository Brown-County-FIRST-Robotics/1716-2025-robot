package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.gripper.Gripper;

public class ManipulatorPanel extends ButtonBoxPanel {
  // Digital Input IDs, assuming that level2 is TROUGH_ID + 1, etc.
  private static final int TROUGH_ID = 0;
  private static final int ALGAE_LOW_ID = 4;
  private static final int LEFT_POLE_ID = 6;
  private static final int UTILITY_ID = 8;
  private static final int EJECT_ID = 9;

  public ManipulatorPanel(ButtonBox bb) {
    super(bb);
  }

  public Trigger trough() { // level 1 coral
    return new Trigger(() -> getButton(TROUGH_ID));
  }

  public Trigger level2() {
    return new Trigger(() -> getButton(TROUGH_ID + 1));
  }

  public Trigger level3() {
    return new Trigger(() -> getButton(TROUGH_ID + 2));
  }

  public Trigger level4() {
    return new Trigger(() -> getButton(TROUGH_ID + 3));
  }

  public Trigger algaeLow() {
    return new Trigger(() -> getButton(ALGAE_LOW_ID));
  }

  public Trigger algaeHigh() {
    return new Trigger(() -> getButton(ALGAE_LOW_ID + 1));
  }

  public Trigger leftPole() {
    return new Trigger(() -> getButton(LEFT_POLE_ID));
  }

  public Trigger rightPole() {
    return new Trigger(() -> getButton(LEFT_POLE_ID + 1));
  }

  // checks if both or neither pole buttons are pressed
  public Trigger noPole() {
    return new Trigger(
        () ->
            (!getButton(LEFT_POLE_ID) && !getButton(LEFT_POLE_ID + 1))
                || (getButton(LEFT_POLE_ID) && getButton(LEFT_POLE_ID + 1)));
  }

  public Trigger processor(Gripper gripper) {
    return new Trigger(() -> getButton(UTILITY_ID) && gripper.hasAlgae());
  }

  public Trigger intake(Gripper gripper) { // coral station
    return new Trigger(() -> getButton(UTILITY_ID) && !gripper.hasAlgae());
  }

  public Trigger eject() {
    return new Trigger(() -> getButton(EJECT_ID));
  }

  @Override
  int getButtons() {
    return 7;
  }

  @Override
  int getAxes() {
    return 0;
  }
}
