// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramseteAuto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import frc.robot.commands.ramseteAuto.SimpleCsvLogger;

public class ReadPath extends CommandBase {
  private final Timer m_timer = new Timer();
  Trajectory trajectory;
  private double m_totalTimeElapsedSec = 0;
  private double m_timeBeforeTrajectorySec = 0;

  // USB Logger
  private SimpleCsvLogger usbLogger;
  private String loggingDataIdentifier;
  
  /** Creates a new ReadPath. */
  public ReadPath( Trajectory trajectoryToRead, String loggingDataIdentifier) {
    
    if ( trajectoryToRead == null)
    {
      System.out.println("input trajectory null");
    }
    else
    {
      System.out.println("input trajectory is valid");
    }
    // Use addRequirements() here to declare subsystem dependencies.
    //trajectory = requireNonNullParam(trajectoryToRead, "Input trajectory", "ReadPath");

    trajectory = trajectoryToRead;
    usbLogger = new SimpleCsvLogger();
    this.loggingDataIdentifier = loggingDataIdentifier;

    System.out.println("ReadPath: trajectory time sec: " + trajectory.getTotalTimeSeconds());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

    m_timeBeforeTrajectorySec = m_timer.get();
    startLogging();
  
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_totalTimeElapsedSec = m_timer.get();
      double curTimeSec = m_totalTimeElapsedSec - m_timeBeforeTrajectorySec;

      Trajectory.State desiredStateSec = trajectory.sample(curTimeSec);

       //log the current state of trajectory
       logData(m_totalTimeElapsedSec,        
       desiredStateSec.poseMeters.getTranslation().getX(), //from trajectory
       desiredStateSec.poseMeters.getTranslation().getY(), 
       desiredStateSec.poseMeters.getRotation().getDegrees(),
       desiredStateSec.velocityMetersPerSecond, 
       desiredStateSec.accelerationMetersPerSecondSq,
       desiredStateSec.curvatureRadPerMeter); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    stopLogging();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(trajectory.getTotalTimeSeconds() < (m_totalTimeElapsedSec - m_timeBeforeTrajectorySec) )
    {
      System.out.println("ReadPath is finished! ");
      System.out.println("ReadPath: trajectory time sec: " + trajectory.getTotalTimeSeconds());
      return true;
    }
    else
        return false;
  }

  public double getMs()
  {
    return RobotController.getFPGATime() / 1000.0;
  }

  /**
   * All Methods used for USB logging startLogging() logData(data) stopLogging()
   */
  public void startLogging() {
      // See Spreadsheet link at top
      usbLogger.init(loggingDataIdentifier, 
      new String[] {
             "TrajectoryTime",
             "SstateX", 
             "SstateY", 
             "SstateRot", //
             "SstateVel", 
             "SstateAccel", 
             "SstateCurv"
           },
      new String[] { "s", 
                     "m", "m", "deg", //
                     "m/s", "m/s/s", "rad/s" //
                    }
     );
  }

  public void logData(double... data) {
      usbLogger.writeData(data);
  }

  public void stopLogging() {
      usbLogger.close();
  }
}
