package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralHandler;

public class ZeroCoralCommand extends SequentialCommandGroup {
    /**
     * Constructs a command to zero the coral mechanism:
     * 1. Moves the arm upward at zeroing power until it hits its upward limit.
     * 2. Stops the arm and commands it to move to ARM_MAX_ANGLE.
     * 3. Commands the elevator to move to position 0.
     */
    public ZeroCoralCommand(CoralHandler coralHandler) {
        addCommands(
            // 1. Move the elevator upward using raw motor output.
            new InstantCommand(() -> coralHandler.getElevator().getMotor().set(CoralConstants.ELEVATOR_ZEROING_POWER_UP), coralHandler),
            new WaitUntilCommand(() -> coralHandler.getElevator().isAtMaxLimit()),

            // 2. Move the arm at zeroing power until it hits its upward limit.
            new InstantCommand(() -> coralHandler.getArm().getMotor().set(CoralConstants.ARM_ZEROING_POWER), coralHandler),
            new WaitUntilCommand(() -> coralHandler.getArm().isAtMaxLimit()),

            // 3. Move the elevator upward using raw motor output.
            new InstantCommand(() -> coralHandler.getElevator().getMotor().set(CoralConstants.ELEVATOR_ZEROING_POWER_DOWN), coralHandler),
            new WaitUntilCommand(() -> coralHandler.getElevator().isAtMinLimit())
        );
    }
}
