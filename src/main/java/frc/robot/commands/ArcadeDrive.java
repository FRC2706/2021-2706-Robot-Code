/*----------------------------------------------------------------------------*/ 
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ 
/* Open Source Software - may be modified and shared by FRC teams. The code   */ 
/* must be accompanied by the FIRST BSD license file in the root directory of */ 
/* the project.                                                               */ 
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.commands.ramseteAuto.SimpleCsvLogger;

/**
 * Drive the robot using values for driving forward and rotation (Arcade Drive)
 */
public class ArcadeDrive extends CommandBase {

  private final Supplier<Double> forwardVal;
  private final Supplier<Double> rotateVal;

  private final boolean squareInputs;
  private final boolean initBrake;

  private final DriveBase driveBase;

  private NetworkTableEntry forwardValue, rotationValue;

  // USB Logger
  private SimpleCsvLogger usbLogger;
  private String loggingDataIdentifier = "ArcadeDrive";
/**
 * Creates the arcade drive
 * 
 * @param forwardVal The values to use for driving forward
 * @param rotateVal the values to use for rotating
 * @param squareInputs Whether or not to square the forward and rotation values
 * @param initBrake whether or not to start and end the command in brake or coast mode
 */
  public ArcadeDrive(Supplier<Double> forwardVal, Supplier<Double> rotateVal, boolean squareInputs, boolean initBrake) {
    // Ensure that this command is the only one to run on the drive base
    // Requires must be included to use this command as a default command for the drive base
    this.forwardVal = forwardVal;
    this.rotateVal = rotateVal;
    this.squareInputs = squareInputs;
    this.initBrake = initBrake;
    this.driveBase = DriveBaseHolder.getInstance();
    addRequirements(this.driveBase);
   
    usbLogger = new SimpleCsvLogger();
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Prepare for driving by human
    this.driveBase.setDriveMode(DriveBase.DriveMode.OpenLoopVoltage);
    this.driveBase.setNeutralMode(initBrake ? NeutralMode.Brake : NeutralMode.Coast);

    var table = NetworkTableInstance.getDefault().getTable("ArcadeDriveData");
    forwardValue = table.getEntry("forwardValue");
    rotationValue = table.getEntry("rotationValue");
    
    this.driveBase.startLogging();
    startLogging();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pass values to drive base to make the robot move
    this.driveBase.arcadeDrive(forwardVal.get(), rotateVal.get(), squareInputs);
  //  System.out.println("Forward value: "+forwardVal.get());
  //  System.out.println("Rotation value: "+rotateVal.get());
    forwardValue.setValue(forwardVal.get());
    rotationValue.setValue(rotateVal.get());

    logData(forwardVal.get(), rotateVal.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Go back to disabled mode
    this.driveBase.setDriveMode(DriveBase.DriveMode.Disabled);

    this.driveBase.stopLogging();
    stopLogging();
  }

  /**
     * All Methods used for USB logging startLogging() logData(data) stopLogging()
     */
    public void startLogging() {
      // See Spreadsheet link at top
      usbLogger.init(loggingDataIdentifier, 
              new String[] { "FVal",
                             "RVal"},
              new String[]{" "," "});
    }

    public void logData(double... data) {
        usbLogger.writeData(data);
    }

    public void stopLogging() {
        usbLogger.close();
    }
}
