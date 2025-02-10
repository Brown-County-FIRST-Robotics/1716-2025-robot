package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManipulatorPanel extends ButtonBoxPanel {
  private boolean doAlgae;

  public ManipulatorPanel(ButtonBox bb) {
    super(bb);
    updateAlgaeSwitch();
  }

  public Trigger leftPole() {
    return new Trigger(() -> getButton(0));
  }

  public Trigger rightPole() {
    return new Trigger(() -> getButton(1));
  }

  public Trigger noPole() { // checks if both or neither pole buttons are pressed
    return new Trigger(
        () ->
            (leftPole().getAsBoolean() && rightPole().getAsBoolean())
                || (!leftPole().getAsBoolean() && !rightPole().getAsBoolean()));
  }

  public Trigger trough() {
    updateAlgaeSwitch();
    return new Trigger(() -> !doAlgae && getButton(2));
  }

  public Trigger level2() {
    updateAlgaeSwitch();
    return new Trigger(() -> !doAlgae && getButton(3));
  }

  public Trigger level3() {
    updateAlgaeSwitch();
    return new Trigger(() -> !doAlgae && getButton(4));
  }

  public Trigger level4() {
    updateAlgaeSwitch();
    return new Trigger(() -> !doAlgae && getButton(5));
  }

  public Trigger algaeLow() {
    updateAlgaeSwitch();
    // bottom two levels work for low algae
    return new Trigger(() -> doAlgae && (trough().getAsBoolean() || level2().getAsBoolean()));
  }

  public Trigger algaeHigh() {
    updateAlgaeSwitch();
    // top two levels work for high algae
    return new Trigger(() -> doAlgae && (level3().getAsBoolean() || level4().getAsBoolean()));
  }

  private void updateAlgaeSwitch() {
    doAlgae = getButton(7);
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
