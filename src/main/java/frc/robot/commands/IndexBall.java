// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.FeederSubsystem;

public class IndexBall extends CommandBase {

    FeederSubsystem feeder; 
    int targetPositon;
    int startPosition;
    int incrementTicks;
    boolean reversing;
    boolean isDone;

    // Whether the command should end next isFinished()
	boolean endCommand;

    /** Creates a new IndexBall. */
    public IndexBall() {
        feeder = FeederSubsystem.getInstance();
		addRequirements(feeder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        endCommand = false;
        if (feeder.getBallsAroundFeeder() >= Config.FEEDER_MAX_BALLS) { 
			endCommand = true;
			// System.out.println("Too many balls around feeder");
            return;
        }

        incrementTicks = Config.FEEDERSUBSYSTEM_INCREMENT_TICKS.get().intValue();
			// System.out.println("incrementTIcks is " + incrementTicks);
        startPosition = (int) feeder.getCurrentPosition();
        targetPositon = startPosition + incrementTicks;
		feeder.setFeederPosistion(targetPositon);
		
        reversing = false;
    }

    @Override
    public void execute() {
        // If the command should end don't do anything extra
        if (endCommand) {
            return;
        }

        int currentPosition = (int) feeder.getCurrentPosition();
		boolean atPosistion = feeder.isFeederAtPosistion(Config.FEEDERSUBSYSTEM_INDEX_ALLOWABLE_ERROR);

        // Check end condition of ball indexed
        if (atPosistion || currentPosition >= targetPositon) {
            // endCommand = true;System.out.println("atpos" + atPosistion + ", curntPose > targetPos: " +  (currentPosition >= targetPositon));
        } 

		// Check if the limit unpressed within the acceptable range to start reversing
		// Check if limit switch exists before asking for a value
		// else if ((Config.FEEDER_SWITCH_INPUT != -1 && feeder.isBallAtInput() == false) && 
		// 		(currentPosition - startPosition) < Config.FEEDERSUBSYSTEM_POS_PAST_SWITCH) {

		// 	reversing = true; System.out.println("reversing = true");
		// }

        // If the limit switch unpressed then reverse
        else if (reversing) {
            if (atPosistion) {
                endCommand = true;
                return;
            } else {
                feeder.setFeederPosistion(startPosition);
            }
        }

        else {
			// System.out.println("Set feeder to " + targetPositon);
            feeder.setFeederPosistion(targetPositon);
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // System.out.println("indexBall interrupted: " + interrupted + " , endCommand: " + endCommand);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Increment the number of balls around the feeder
        if (endCommand && reversing == false) {
            feeder.setBallsAroundFeeder(feeder.getBallsAroundFeeder()+1);
        }
        return endCommand;
    }
}
