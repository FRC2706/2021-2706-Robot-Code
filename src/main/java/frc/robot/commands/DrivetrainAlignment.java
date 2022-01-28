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
  double m_initDegree;
  double m_targetDegree;

  double m_targetDeltaPositionMeter;
  double m_targetLeftPositionMeter;
  double m_targetRightPositionMeter;
  double m_currLeftPosMeter;
  double m_currRightPosMeter;

  // Get the drivebase and pigeon
  private final DriveBase m_drivebase;
  
  //A timer to ensure the command doesn't get stuck and the robot cannot drive
  private Timer m_timer;
  double m_timeout;

  private boolean bDone;
  private boolean bTimeouted;
  private final double m_errMeters = 0.01;// 1cm

  public DrivetrainAlignment(double deltaDegree) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_deltaDegree = deltaDegree;

    m_drivebase = DriveBaseHolder.getInstance();
    addRequirements(m_drivebase);

    m_timer = new Timer();
    m_timeout = 1.0;//0.5;//0.25;        //seconds //@todo: from config
    
    System.out.println("DrivertrainAlignemnt construct");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    covertDegreeToPositionMeter();

    //Note: always counter-clockwise rotation. Keep this in mind when calcualte target angle
    m_targetLeftPositionMeter  = m_drivebase.getLeftPosition() + m_targetDeltaPositionMeter;
    m_targetRightPositionMeter = m_drivebase.getRightPosition() - m_targetDeltaPositionMeter;

    m_initDegree = m_drivebase.getOdometryHeading().getDegrees();
    m_targetDegree = m_initDegree + m_deltaDegree;

    //setup PID slot of two master talons
    m_drivebase.setActivePIDSlot(Config.DRIVETRAIN_SLOTID_ALIGNMENT);
    m_drivebase.setCoastMode();

    m_timer.start();
    //todo: this reset has to be added.
    m_timer.reset();
    bDone = false;
    bTimeouted = false;

    System.out.println("DrivertrainAlignemnt initialize " + m_targetLeftPositionMeter  +" "+ m_targetRightPositionMeter );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Since targe position is fixed, we can set it multiple times
    m_drivebase.tankDrivePosition( m_targetLeftPositionMeter, m_targetRightPositionMeter);

    //get the current encoder positions
    m_currLeftPosMeter =  m_drivebase.getLeftPosition();
    m_currRightPosMeter = m_drivebase.getRightPosition();

    if ( (Math.abs(m_currLeftPosMeter - m_targetLeftPositionMeter) < m_errMeters )
         && ( Math.abs(m_currRightPosMeter - m_targetRightPositionMeter) < m_errMeters ))
    {
      bDone = true;
    }

  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivebase.setActivePIDSlot(Config.DRIVETRAIN_SLOTID_DRIVER);

    //logging the final current heading and error angle
    double finalDegree = m_drivebase.getOdometryHeading().getDegrees();
    double errDegree = finalDegree - m_targetDegree;
    System.out.println("current odometry angle (degrees): "+ finalDegree);
    System.out.println("target odometry angle (degrees): " + m_targetDegree);
    System.out.println("error angle (degrees): "+ errDegree);
    m_timer.stop();

    //@todo: after target position is reached, stop the cmd.
    //@max velocity, trapezoid control
    System.out.println("curr left pos: "+ m_currLeftPosMeter+" target pos: "+ m_targetLeftPositionMeter);
    System.out.println("curr right pos: "+ m_currRightPosMeter+" target pos: "+ m_targetRightPositionMeter);
    System.out.println("timeout: " + bTimeouted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ( m_timer.get() > m_timeout )
    {
      bTimeouted = true;
    }

    if( m_timer.get() > m_timeout || bDone == true)
      return true;
    else
      return false;
  }

  public void covertDegreeToPositionMeter()
  {
    //@todo: convert m_deltaDegree to m_targetPositionMeter based on radius.
    m_targetDeltaPositionMeter = 0.246;  //mapped to 90 degrees
  }

}
