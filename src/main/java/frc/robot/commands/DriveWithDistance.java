/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBase.DistanceType;


public class DriveWithDistance extends CommandBase {

//Initializing private version of drivebase object
private final DriveBase driveBase;

// initialization of the variables 
  public double currentRightDistance = 0;
  public double currentLeftDistance = 0;
  public double desiredLeftDistance = 0;
  public double desiredRightDistance = 0;
  public double encoderTicks;
  public double leftSpeed = 0;
  public double rightSpeed = 0;
  public DistanceType currentType;
  public double currentConversion;
  public boolean backWards = false;
  public boolean commandFinished = false;

  /**
   * Constructor to be used when the input specifications do not provide the right and left speed (assumed to be 0.5)
   * 
   * @param distance The distance to drive
   * @param currentType The distance unit type to drive in
   * @param rightSpeed The right motor speed
   * @param leftSpeed The left motor speed
   */

  public DriveWithDistance(double distance, DistanceType currentType) {

    //If you have to move backwards
    if(distance < 0){
      //Error since you can't move backwards with positive speeds (0.5 is default in this constructor)
      commandFinished = true;

    }

    //Setting current unit type and drive base variables
    this.currentType = currentType;
    this.driveBase = DriveBase.getInstance();
    addRequirements(driveBase);
    
    //Setting the current conversion type according to the chosen distance unit
    switch (currentType) {
      case FEET:
        this.currentConversion = DriveBase.DistanceUnit.FEET.encoderTicks;    
        
        break;
        case METERS:
        this.currentConversion = DriveBase.DistanceUnit.METERS.encoderTicks;    
        
        break;
        case CENTIMETERS:
        this.currentConversion = DriveBase.DistanceUnit.CENTIMETERS.encoderTicks;    
        
        break;
        case INCHES:
        this.currentConversion = DriveBase.DistanceUnit.INCHES.encoderTicks;    
        
        break;

      //Default is meters
      default:

        this.currentConversion = DriveBase.DistanceUnit.METERS.encoderTicks;   
        break;
        
    }

    //default speed is 0.5 for the left and right talons
    rightSpeed = 0.5;
    leftSpeed = 0.5;
    this.encoderTicks = distance * this.currentConversion;   
  }

  /**
   * Constructor to be used when the input gives the right and left distance
   * 
   * @param distance The distance to drive
   * @param currentType The distance unit type to drive in
   * @param rightSpeed The right motor speed
   * @param leftSpeed The left motor speed
   */
  public DriveWithDistance(double distance, DistanceType currentType, double rightSpeed, double leftSpeed) {


    //If all the values for distance and speeds are the same magnitude (+/-) then run the robot, otherwise do not.
    if(distance < 0 && rightSpeed < 0 && leftSpeed < 0){
      backWards = true;
    }
    else if(distance < 0 && rightSpeed < 0 && leftSpeed < 0){
      backWards = false;
    }
    //The robot should not move if it has inverse speeds and desired distances since that must be an error
    else{
      commandFinished = true;
    }

   

    //Setting current distance unit
    this.currentType = currentType;

    //Initating drivebase
    this.driveBase = DriveBase.getInstance();
    addRequirements(driveBase);
    
    //Setting the current conversion type according to the chosen distance unit
    switch (currentType) {
      case FEET:
        this.currentConversion = DriveBase.DistanceUnit.FEET.encoderTicks;    
        
        break;
        case METERS:
        this.currentConversion = DriveBase.DistanceUnit.METERS.encoderTicks;    
        
        break;
        case CENTIMETERS:
        this.currentConversion = DriveBase.DistanceUnit.CENTIMETERS.encoderTicks;    
        
        break;
        case INCHES:
        this.currentConversion = DriveBase.DistanceUnit.INCHES.encoderTicks;    
        
        break;

      //Default is meters
      default:

        this.currentConversion = DriveBase.DistanceUnit.METERS.encoderTicks;   
        break;
        
    }

    //Setting the right speed and left speed according to input specifications
    this.rightSpeed = rightSpeed;
    this.leftSpeed = leftSpeed;

    //Determing the amount of desired encoder ticks to drive by getting the distnace and multiplying by the chosen distance unit
    this.encoderTicks = distance * this.currentConversion;   
    

  }

  
  /**
   * Called  one when the command is initially scheduled and determines the desired distance to travel
   * 
   */
  @Override
  public void initialize() {

    //Add the desired amount of encoder ticks to the current distance to get the amount of encoder ticks that the robot needs to drive
    desiredRightDistance = driveBase.getRightDistance() + encoderTicks;
    addRequirements(DriveBase.getInstance());

    //Dashboard
    SmartDashboard.putBoolean("Running", true);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. (run every 20ms) and determines the current distance
   * 
   */
  @Override
  public void execute() {
   
    //Get the current distance of the right encoder and store value in variable
    double currentRightDistance = DriveBase.getInstance().getRightDistance();

    //Run motors according to output of the speeds
    driveBase.tankDrive(leftSpeed, rightSpeed, false);

    //Dashboard
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);
    SmartDashboard.putNumber("Left Error", desiredLeftDistance - currentLeftDistance);
    SmartDashboard.putNumber("Right Error", desiredRightDistance - currentRightDistance);
    
  }

  
  
  /**
   * Determines when the command is finished using the current distnce and desired distance
   * 
   */

  @Override
  public boolean isFinished() {
    
    //Update to the current right distance that the robot has driven
    currentRightDistance = DriveBase.getInstance().getRightDistance();

    //If the robot has reached or surpassed the desired distance, then stop the robot. Otherwise, keep moving (moving forward). 
    if(!backWards && currentRightDistance >= desiredRightDistance){
        driveBase.tankDrive(0, 0, false);
        commandFinished = true;

    }
    //If the robot has reached or surpassed the desired distance, then stop the robot. Otherwise, keep moving (moving backward).
    else if(backWards && currentRightDistance <= desiredRightDistance){
        driveBase.tankDrive(0, 0, false);
        commandFinished = true;
    }
    else{
        commandFinished = false;
    }


    return commandFinished;
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Running", false);
    }
  
}
