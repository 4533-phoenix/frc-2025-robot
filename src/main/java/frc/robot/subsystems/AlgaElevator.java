package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaElevatorConstants;

/** Subsystem that controls the Alga Elevator position mechanism. */
public class AlgaElevator extends SubsystemBase {
  private static AlgaElevator instance;

  public static AlgaElevator getInstance() {
    if (instance == null) {
      instance = new AlgaElevator();
    }
    return instance;
  }

  private final VictorSPX retractionMotor;
  private final DigitalInput upperLimit;
  private final DigitalInput lowerLimit;
  private final NetworkTable table;

  /** Enum representing the position state of the Alga Elevator. */
  public enum ElevatorState {
    DOWN,
    UP,
    UNKNOWN
  }

  private ElevatorState elevatorState = ElevatorState.UNKNOWN;

  /** Trigger that is active when the upper limit switch is pressed. */
  public final Trigger atUpperLimit;

  /** Trigger that is active when the lower limit switch is pressed. */
  public final Trigger atLowerLimit;

  private AlgaElevator() {
    retractionMotor = new VictorSPX(AlgaElevatorConstants.RETRACTION_CAN_ID);
    upperLimit = new DigitalInput(AlgaElevatorConstants.UPPER_LIMIT_CHANNEL);
    lowerLimit = new DigitalInput(AlgaElevatorConstants.LOWER_LIMIT_CHANNEL);
    table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("AlgaElevator");

    atUpperLimit = new Trigger(() -> !upperLimit.get());
    atLowerLimit = new Trigger(() -> !lowerLimit.get());
  }

  /**
   * Toggles the elevator position between UP and DOWN states.
   *
   * @return A command that toggles the elevator position.
   */
  public Command togglePosition() {
    return runOnce(() -> {
      ElevatorState startingState = elevatorState;
      double power = switch (elevatorState) {
        case DOWN -> AlgaElevatorConstants.RETRACTION_POWER;
        case UP -> -AlgaElevatorConstants.RETRACTION_POWER;
        case UNKNOWN -> AlgaElevatorConstants.RETRACTION_POWER; // Default to retracting
      };

      run(() -> retractionMotor.set(VictorSPXControlMode.PercentOutput, power))
          .until(() -> elevatorState != startingState)
          .andThen(runOnce(() -> retractionMotor.set(VictorSPXControlMode.PercentOutput, 0)))
          .schedule();
    }).withName("ToggleElevatorPosition");
  }

  /**
   * Gets the current elevator state.
   *
   * @return The current elevator state.
   */
  public ElevatorState getElevatorState() {
    return elevatorState;
  }

  @Override
  public void periodic() {
    updateElevatorState();
    updateNetworkTables();
  }

  private void updateElevatorState() {
    if (atUpperLimit.getAsBoolean()) {
      elevatorState = ElevatorState.UP;
    } else if (atLowerLimit.getAsBoolean()) {
      elevatorState = ElevatorState.DOWN;
    }
    // Once a limit is reached, never return to UNKNOWN
  }

  private void updateNetworkTables() {
    table.getEntry("elevatorState").setString(elevatorState.name());
    table.getEntry("upperLimit").setBoolean(atUpperLimit.getAsBoolean());
    table.getEntry("lowerLimit").setBoolean(atLowerLimit.getAsBoolean());
  }
}
