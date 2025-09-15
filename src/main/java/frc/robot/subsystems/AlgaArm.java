package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaArmConstants;

/** Subsystem that controls the Alga Arm, responsible for intaking and dropping Alga game pieces. */
public class AlgaArm extends SubsystemBase {
  /** Singleton instance of the AlgaArm subsystem. */
  private static AlgaArm instance;

  /**
   * Gets the singleton instance of the AlgaArm subsystem.
   *
   * @return The singleton instance.
   */
  public static AlgaArm getInstance() {
    if (instance == null) {
      instance = new AlgaArm();
    }
    return instance;
  }

  /** Motor controller for the alga intake mechanism. */
  private SparkMax algaMotor;

  /** Limit switch for detecting alga game pieces. */
  private DigitalInput algaSensor;

  /** NetworkTable for publishing alga detection state. */
  private NetworkTable table;

  private AlgaArm() {
    algaMotor = new SparkMax(AlgaArmConstants.CAN_ID, MotorType.kBrushless);
    algaSensor = new DigitalInput(AlgaArmConstants.SENSOR_CHANNEL);
    table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("AlgaArm");

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(AlgaArmConstants.ALGA_INVERTED);
    sparkMaxConfig.smartCurrentLimit(20);
    sparkMaxConfig.openLoopRampRate(0.1);
    sparkMaxConfig.closedLoopRampRate(0.1);
    algaMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    setDefaultCommand(stop());
  }

  /** Trigger that is active when the Alga Arm does not have an Alga. */
  public final Trigger doesntHaveAlga = new Trigger(() -> !algaSensor.get()).debounce(0.25);

  /** Trigger that is active when the Alga Arm has an Alga. */
  public final Trigger hasAlga = new Trigger(() -> algaSensor.get());

  /** Trigger that is active when the Alga Arm is running. */
  public final Trigger running =
      new Trigger(
          () -> {
            return algaMotor.get() != 0;
          });

  /**
   * Runs the intake at a constant power.
   *
   * @return A command that runs the intake.
   */
  public Command runIntake() {
    return run(() -> algaMotor.set(AlgaArmConstants.INTAKE_POWER)).withName("algaRunIntake");
  }

  /**
   * Intakes an alga until the sensor detects it.
   *
   * @return A command that intakes an alga.
   */
  public Command intake() {
    return run(() -> algaMotor.set(AlgaArmConstants.INTAKE_POWER))
        .onlyWhile(doesntHaveAlga)
        .withName("algaIntake");
  }

  /**
   * Runs the drop at a constant power.
   *
   * @return A command that runs the drop.
   */
  public Command runDrop() {
    return run(() -> algaMotor.set(AlgaArmConstants.DROP_POWER)).withName("algaRunDrop");
  }

  /**
   * Drops an alga until the sensor no longer detects it.
   *
   * @return A command that drops an alga.
   */
  public Command drop() {
    return run(() -> algaMotor.set(AlgaArmConstants.DROP_POWER))
        .onlyWhile(hasAlga)
        .withName("algaDrop");
  }

  /**
   * Releases the alga with a short drop.
   *
   * @return A command that releases the alga.
   */
  public Command release() {
    return drop()
        .withTimeout(0.1)
        .andThen(
            run(() -> {
                  algaMotor.set(0.0);
                })
                .withTimeout(0.1));
  }

  /**
   * Stops the alga arm.
   *
   * @return A command that stops the alga arm.
   */
  public Command stop() {
    return run(() -> algaMotor.set(0)).withName("algaStop");
  }

  /**
   * Toggles between intake and drop based on the current state.
   *
   * @return A command that toggles the alga arm.
   */
  public Command toggle() {
    return runOnce(
            () -> {
              if (running.getAsBoolean()) {
                stop().schedule();
              } else {
                if (hasAlga.getAsBoolean()) {
                  drop().schedule();
                } else {
                  intake().schedule();
                }
              }
            })
        .withName("toggle");
  }

  @Override
  public void periodic() {
    table.getEntry("running").setBoolean(running.getAsBoolean());
    table.getEntry("hasAlga").setBoolean(hasAlga.getAsBoolean());
  }
}