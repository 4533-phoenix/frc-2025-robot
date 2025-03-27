package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot initialization class that starts the robot code execution. This class serves as the entry
 * point for the robot program and handles the initialization of the robot runtime.
 *
 * <p>This class should not be modified except to change the Robot class passed to startRobot. Any
 * robot-specific initialization or configuration should be done in the Robot class.
 *
 * <p>The main method is called by the JVM and initializes the WPILib framework, which then manages
 * the robot's execution lifecycle.
 */
public final class Main {
  /**
   * Private constructor to prevent instantiation. This class should not be instantiated since it
   * only contains a static main method.
   */
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   *
   * @param args command line arguments (not used)
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::getInstance);
  }
}
