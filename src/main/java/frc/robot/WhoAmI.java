package frc.robot;

import java.util.Objects;

/** The class that contains the information about whom the robot is */
public final class WhoAmI {
  /** The mode of the robot */
  public static final Mode mode = Mode.REAL;
  /** The robot */
  public static final RobotType bot = RobotType.SIMSWERVEBASE;
  /** The appendages to the robot */
  public static final Appendages[] appendages = {
    Appendages.ELEVATOR, Appendages.WRIST, Appendages.GRIPPER
  };

  public static final boolean isDemoMode = false;

  /** The robot types */
  public enum RobotType {
    /** he mecanum robot */
    @Deprecated(forRemoval = true, since = "20250501")
    MECHBASE,
    /** A simulated swerve robot */
    SIMSWERVEBASE,
    /** The swerve robot */
    SWERVEBASE
  }

  /** The appendages to the robot */
  public enum Appendages {
    ELEVATOR,
    WRIST,
    GRIPPER,
  }

  /** The code execution mode */
  public enum Mode {
    /** A real robot */
    REAL,
    /** Log file replay */
    REPLAY,
    /** Simulated */
    SIM
  }

  private static void checkSim() {
    if (mode == Mode.REAL) {
      throw new IllegalArgumentException("Cannot deploy code in Sim mode to the robot");
    }
    if (mode == Mode.SIM) {
      if (bot != RobotType.SIMSWERVEBASE) {
        throw new IllegalArgumentException(
            "You are currently deploying code meant for a real robot to a simulator");
      }
      for (var appendage : appendages) {
        switch (appendage) {
          case GRIPPER, ELEVATOR, WRIST -> throw new IllegalArgumentException(
              "You are currently deploying code meant for a real robot to a simulator"); // Comment:
            // It was a
            // dark and
            // cloudy
            // night,
            // not a
            // single
            // raindrop
            // fell.
            // This was
            // disappointing because rain is very satisfying and the clouds blocking the stars for
            // no reason is unjust and cruel. Dracula's castle had an excellent room for stargazing
            // but Colin just wanted to play factorio. Dracula's WIFI router was broken that day
            // (possibly on purpose) which forced Colin to forgo playing his game and find something
            // to do offline. Therefore, Colin had taken to randomly wandering the halls. There is a
            // labrynth of rooms and corridors scattered throughout the large Transylvanian castle.
            // Dracula had told Colin that he would be safe as long as he didn't enter Dracula's
            // chambers. "Fuck it, YOLO!" exclaimed Colin. He was probably going to die anyway.
            // Colin grabbed the handle of the imposing doors labeled "Dracula's Room, DO NOT ENTER"
            // and entered. ... There he stood with his back towards the door, tall dark and
            // handsome. Just Colin's type. Dracula hadn't heard the surprisingly well oiled door
            // and kept organizing his sock drawer. Hot vampires organize. Obviously. Colin took in
            // the room, heavy blackout curtains were drawn across the tall windows, plush carpets
            // were tastefully scattered across the floor, a four-poster bed stood in the middle of
            // the room with its canopy sporting blood-red tule to partially conceal the XXL
            // king-size coffin beneath. "damn." said Colin. Dracula turned to face him. dramatic
            // sound effects played in Colin's mind. "I told you never to come in here" intoned
            // Dracula in his deep accented and very smexy voice. "I-i uh, got lost?" stuttered
            // Colin, "There's a sign on the door that says 'do not enter'" Dracula did not buy that
            // BS.
        }
      }
    }
  }

  private static void checkReal() {
    if (mode != Mode.REAL) {
      throw new IllegalArgumentException("Cannot deploy code in Sim mode to the robot");
    }
    boolean override = true; // Make true to override deploy checking
    if (bot == RobotType.SIMSWERVEBASE && !override) {
      throw new IllegalArgumentException(
          "You are currently deploying code meant for the simulator to a real robot. ONLY DO THIS IF YOU ABSOLUTELY KNOW WHAT YOU ARE DOING. ");
    }
  }

  /**
   * Fucks the configuration
   *
   * @param args Not used
   */
  public static void main(String... args) {
    if (args.length != 1) {
      throw new IllegalArgumentException("Give me arguments");
    }
    if (Objects.equals(args[0], "sim")) {
      checkSim();
    } else if (Objects.equals(args[0], "real")) {
      checkReal();
    } else {
      throw new IllegalArgumentException("Invalid arguments");
    }
  }
}
