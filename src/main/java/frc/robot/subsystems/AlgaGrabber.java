package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaGrabberConstants;

/** Subsystem that controls the Alga Grabber motor mechanism. */
public class AlgaGrabber extends SubsystemBase {
  private static AlgaGrabber instance;

  public static AlgaGrabber getInstance() {
    if (instance == null) {
      instance = new AlgaGrabber();
    }
    return instance;
  }

  private final SparkMax grabberMotor;
  private final NetworkTable table;
  private final AlgaElevator elevator;

  private AlgaGrabber() {
    grabberMotor = new SparkMax(AlgaGrabberConstants.CAN_ID, MotorType.kBrushless);
    elevator = AlgaElevator.getInstance();
    table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("AlgaGrabber");

    configureMotor();
    setDefaultCommand(stop());
  }

  private void configureMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(AlgaGrabberConstants.INVERTED);
    config.smartCurrentLimit(20);
    config.openLoopRampRate(0.1);
    config.closedLoopRampRate(0.1);
    grabberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Runs the intake motor when elevator is in DOWN position.
   *
   * @return A command that runs the intake.
   */
  public Command runIntake() {
    return run(() -> grabberMotor.set(AlgaGrabberConstants.INTAKE_POWER))
        .onlyIf(elevator.atLowerLimit)
        .onlyWhile(elevator.atLowerLimit)
        .withName("AlgaIntake");
  }

  /**
   * Runs the drop motor when elevator is in DOWN position.
   *
   * @return A command that runs the drop.
   */
  public Command runDrop() {
    return run(() -> grabberMotor.set(AlgaGrabberConstants.DROP_POWER))
        .onlyIf(elevator.atLowerLimit)
        .onlyWhile(elevator.atLowerLimit)
        .withName("AlgaDrop");
  }

  /**
   * Stops the grabber motor.
   *
   * @return A command that stops the motor.
   */
  public Command stop() {
    return runOnce(() -> grabberMotor.set(0)).withName("AlgaStop");
  }

  @Override
  public void periodic() {
    updateNetworkTables();
  }

  private void updateNetworkTables() {
    table.getEntry("motorOutput").setDouble(grabberMotor.get());
    table.getEntry("canRunMotor").setBoolean(elevator.atLowerLimit.getAsBoolean());
  }
}
