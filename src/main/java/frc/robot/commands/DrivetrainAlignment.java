// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;
import frc.robot.config.Config;

public class DrivetrainAlignment extends CommandBase {
  /** Creates a new DrivetrainAlignment. */
  double m_deltaDegree;
  double m_targetPositionMeter;
  double m_targetLeftEncoderPositionMeter;
  double m_targetRightEncoderPositionMeter;

  // Get the drivebase and pigeon
  private final DriveBase m_drivebase;
  
  //A timer to ensure the command doesn't get stuck and the robot cannot drive
  private Timer m_timer;
  double m_timeout;

  public DrivetrainAlignment(double deltaDegree) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_deltaDegree = deltaDegree;

    m_drivebase = DriveBaseHolder.getInstance();
    addRequirements(m_drivebase);

    m_timer = new Timer();
    m_timeout = 2;        //seconds //@todo: from config
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    covertDegreeToPositionMeter();

    //Note: always counter-clockwise rotation. Keep this in mind when calcualte target angle
    m_targetLeftEncoderPositionMeter  = m_drivebase.getLeftEncoderPosition() + m_targetPositionMeter;
    m_targetRightEncoderPositionMeter = m_drivebase.getRightEncoderPosition() - m_targetPositionMeter;

    //setup PID slot of two master talons
    m_drivebase.setActivePIDSlot(Config.DRIVETRAIN_SLOTID_RAMSETE);
    m_drivebase.setCoastMode();

    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivebase.tankDrivePosition( m_targetLeftEncoderPositionMeter, m_targetRightEncoderPositionMeter);
   
    //Q: do we need to call this periodically or just one time call?
    //add a timeout for 2 seconds?

  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.setActivePIDSlot(Config.DRIVETRAIN_SLOTID_DRIVER);

    //@todo: logging the current heading
    //compared with the target angle
    System.out.println("current odometry angle (degrees)"+ m_drivebase.getOdometryHeading().getDegrees());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if( m_timer.get() > m_timeout )
      return true;
    else
      return false;
  }

  public void covertDegreeToPositionMeter()
  {
    //@todo: convert m_deltaDegree to m_targetPositionMeter based on radius.
    m_targetPositionMeter = 0.2; //
  }

}
