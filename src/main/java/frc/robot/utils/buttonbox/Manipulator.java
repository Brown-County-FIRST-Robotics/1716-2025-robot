package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Manipulator extends ButtonBoxPanel {
  public Manipulator(ButtonBox bb) {
    super(bb);
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
    return new Trigger(() -> getButton(2));
  }

  public Trigger level2() {
    return new Trigger(() -> getButton(3));
  }

  public Trigger level3() {
    return new Trigger(() -> getButton(4));
  }

  public Trigger level4() {
    return new Trigger(() -> getButton(5));
  }

  public Trigger algaeLow() {
    // bottom two levels work for low algae
    return new Trigger(() -> trough().getAsBoolean() || level2().getAsBoolean());
  }

  public Trigger algaeHigh() {
    // top two levels work for high algae
    return new Trigger(() -> level3().getAsBoolean() || level4().getAsBoolean());
  }

  public Trigger collectAlgae() {
    return new Trigger(() -> getButton(7));
  }

  public Trigger placeCoral() {
    return new Trigger(
        () -> !collectAlgae().getAsBoolean()); // same button as collectAlgae, just inverted
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
