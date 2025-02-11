package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManipulatorPanel extends ButtonBoxPanel {
  // Digital Input IDs, assuming that level2 is TROUGH_ID + 1, etc.
  private static final int TROUGH_ID = 0;
  private static final int LEFT_POLE_ID = 4;
  private static final int EJECT_ID = 6;

  public ManipulatorPanel(ButtonBox bb) {
    super(bb);
  }

  public Trigger trough() { // level 1 coral
    return new Trigger(
        () ->
            getButton(TROUGH_ID)
                && !getButton(TROUGH_ID + 1)); // only activate if algaeLow won't activate
  }

  public Trigger level2() {
    return new Trigger(() -> getButton(TROUGH_ID + 1) && !getButton(TROUGH_ID));
  }

  public Trigger level3() {
    return new Trigger(() -> getButton(TROUGH_ID + 2) && !getButton(TROUGH_ID + 3));
  }

  public Trigger level4() {
    return new Trigger(() -> getButton(TROUGH_ID + 3) && !getButton(TROUGH_ID + 2));
  }

  public Trigger algaeLow() {
    // press both bottom buttons for low algae
    return new Trigger(() -> getButton(TROUGH_ID) && getButton(TROUGH_ID + 1));
  }

  public Trigger algaeHigh() {
    // press both top buttons for high algae
    return new Trigger(() -> getButton(TROUGH_ID + 2) && getButton(TROUGH_ID + 3));
  }

  public Trigger leftPole() {
    return new Trigger(
        () ->
            getButton(LEFT_POLE_ID)
                && !getButton(LEFT_POLE_ID + 1)); // only activate if not both are pressed
  }

  public Trigger rightPole() {
    return new Trigger(() -> getButton(LEFT_POLE_ID + 1) && !getButton(LEFT_POLE_ID));
  }

  // checks if neither pole buttons are pressed
  public Trigger noPole() {
    return new Trigger(() -> !getButton(LEFT_POLE_ID) && !getButton(LEFT_POLE_ID + 1));
  }

  public Trigger processor() {
    // TODO: check whether the robot has an algae or not to determine whether to do processor or
    // intake
    return new Trigger(() -> getButton(LEFT_POLE_ID) && getButton(LEFT_POLE_ID + 1));
  }

  public Trigger intake() { // coral station
    return new Trigger(() -> getButton(LEFT_POLE_ID) && getButton(LEFT_POLE_ID + 1));
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
