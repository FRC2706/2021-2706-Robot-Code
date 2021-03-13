package frc.robot.commands;

import frc.robot.config.Config;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nettables.VisionCtrlNetTable;

/**
 * VisionAssistedShooter command.
 * NOTE: this command should be run after command TurnToOuterPortCommand
 */
public class VisionAssistedShooter extends CommandBase {
  
  //subsystem
  private final ShooterSubsystem shooter;
  private Timer shooterTimer; 
  private int shooterTime;
  
  //todo: can be configured in config file as well
  //todo: measure the radius for the shooting wheel
  private final double SHOOTER_ANGLE_IN_DEGREES  = 60.0;
  private final double TARGET_HEIGHT_IN_METERS = 2.49;
  private final double SHOOTER_WHEEL_RADIUS_IN_CM = 10;
  private final double HALF_OF_GRAVITY = 4.91;
  private final double CONVERSION_NUMBER = 3000;
  private final int MAX_RPM = 2000;

  // values from Vision Network Table
  private double distanceToOuterPort;
   
  //network table for vision control
  private VisionCtrlNetTable visionControlNetTable;

  //calculated for the shooter
  double targetDistance = 0; 
  double targetRPM = 0;


  /**
   * Creates a new VisionAssistedShooter Command.
   *
   * @param time represents how long the command takes to run
   */
  public VisionAssistedShooter(int time) {
   
    //subsystem
    shooter = ShooterSubsystem.getInstance();
    shooterTimer = new Timer();
    shooterTime = time;
   
    // Use addRequirements() here to declare subsystem dependencies.
    if (shooter.isActive()) {
      addRequirements(shooter);
    }
  }

  @Override
  public void initialize() {

    shooterTimer.start();

    // Ensure the vision is running in tape mode
    visionControlNetTable.setTapeMode();

    //Read the network table from vision to get the distance from the power port.
    //distance from vision
    //NOTE: unit should be meter. If not, need conversion here.
    distanceToOuterPort = visionControlNetTable.distanceToOuterPort.get();

    //todo: to adjuste the distance for the shooter
    //Check the source location of vision distance to outer port 
    //targetDistance

    //Calculate the RPM of the shooter wheel.
    double targetV = initVelocity(distanceToOuterPort);
    targetRPM = velocityToRPM(targetV);

    SmartDashboard.putNumber("Vision: distance to outer port", distanceToOuterPort);
    SmartDashboard.putNumber("Target RPM", targetRPM);

    
  }

  @Override
  public void execute() {

    //Set the shooter to the target RPM.
    shooter.setTargetRPM((int) targetRPM);
        
  }

  @Override
  public boolean isFinished() {
      // This command should only be run once
      return shooterTimer.get() > shooterTime;
  }

  @Override
    public void end(boolean interrupted) {
        shooter.setTargetRPM(0);
    }


  double initVelocity(double distanceToTargetInMeters) {
    double dCheck = Math.tan(SHOOTER_ANGLE_IN_DEGREES)*distanceToTargetInMeters - TARGET_HEIGHT_IN_METERS;
    double dTemp;

    //unit: m/s
    double dInitVelocity;
    if (dCheck > 0)
    {
         dTemp = Math.sqrt(HALF_OF_GRAVITY/dCheck);
         dInitVelocity = distanceToTargetInMeters/Math.cos(SHOOTER_ANGLE_IN_DEGREES) * dTemp;
    }
    else
    {
         dInitVelocity = 0.0;
         System.out.println("WARNING! Not suitable for shooting!");      
     }

    return dInitVelocity;
  
 }

 // convert velocity to RPM
 // velocity: unit m/s
 // return: unit revolutions per minute
double velocityToRPM( double velocity)
 {     
     double rpm = velocity*CONVERSION_NUMBER/(Math.PI*SHOOTER_WHEEL_RADIUS_IN_CM);
     if(rpm > MAX_RPM){
      System.out.println("WARNING! Unsafe RPM reached "+rpm);
      rpm = MAX_RPM;
     }
     return rpm;
 }

}