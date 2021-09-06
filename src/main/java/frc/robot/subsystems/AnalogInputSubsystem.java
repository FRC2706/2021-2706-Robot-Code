// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.AnalogInput;
import java.util.logging.Logger;

import frc.robot.config.Config;

public class AnalogInputSubsystem extends SubsystemBase {

  private static AnalogInputSubsystem single_instance;

  private AnalogInput inputMB1013;
  private AnalogInput inputMB1043;
  private AnalogInput inputInfrared2Y;
  private AnalogInput inputInfrared0A;

  private final AnalogPotentiometer m_ultraSonic_MB1043;
  private final AnalogPotentiometer m_ultraSonic_MB1013;
  private final AnalogPotentiometer m_Infrared_2Y;
  private final AnalogPotentiometer m_Infrared_0A;

  public double m_currDistanceMB1043CM;
  public double m_currDistanceMB1013CM;
  public double m_currDistanceInfrared2Y;
  public double m_currDistanceInfrared0A;

  private NetworkTable analogInputSensorTable;
  private NetworkTableEntry distanceMB1013Entry;
  private NetworkTableEntry distanceMB1043Entry;
  private NetworkTableEntry distanceInfrared2YEntry;
  private NetworkTableEntry distanceInfrared0AEntry;

  private Logger logger = Logger.getLogger("AnalogInputSubsystem");

  public static AnalogInputSubsystem getInstance()
  {
    if (single_instance == null )
      single_instance = new AnalogInputSubsystem();
    
      return single_instance;
  }

  /** Creates a new AnalogInputSubsystem. */
  private AnalogInputSubsystem() {
    inputMB1043 = new AnalogInput(Config.MINIROBOT_MB1043_ANALOG_PORT);
    m_ultraSonic_MB1043 = new AnalogPotentiometer(inputMB1013, 
                                                  Config.MINIROBOT_MBUltraSound_RANGE_CM, 
                                                  Config.MINIROBOT_MBUltraSound_MIN_CM);
    
    
    inputMB1013 = new AnalogInput(Config.MINIROBOT_MB1013_ANALOG_PORT);
    m_ultraSonic_MB1013 = new AnalogPotentiometer(inputMB1013, 
                                                  Config.MINIROBOT_MBUltraSound_RANGE_CM, 
                                                  Config.MINIROBOT_MBUltraSound_MIN_CM);

    inputInfrared2Y = new AnalogInput(Config.MINIROBOT_2Y0A02_ANALOG_PORT);
    m_Infrared_2Y   = new AnalogPotentiometer(inputInfrared2Y,
                                              Config.MINIROBOT_INFRARED2Y_RANGE_CM,
                                              Config.MINIROBOT_INFRARED2Y_MIN_CM);

    inputInfrared0A = new AnalogInput(Config.MINIROBOT_0A41SK_ANALOG_PORT);
    m_Infrared_0A   = new AnalogPotentiometer(inputInfrared0A,
                                              Config.MINIROBOT_INFRARED0A_RANGE_CM,
                                              Config.MINIROBOT_INFRARED0A_MIN_CM);
    //setup network table entries
    analogInputSensorTable  = NetworkTableInstance.getDefault().getTable("Analog Input Sensors");
    distanceMB1013Entry     = analogInputSensorTable.getEntry("MB1013_distance(cm)");
    distanceMB1043Entry     = analogInputSensorTable.getEntry("MB1043_distance(cm)");
    distanceInfrared2YEntry = analogInputSensorTable.getEntry("Infrared2Y_distance(cm)");
    distanceInfrared0AEntry = analogInputSensorTable.getEntry("Infrared0A_distance(cm)");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_currDistanceMB1043CM = m_ultraSonic_MB1043.get();
    m_currDistanceMB1013CM = m_ultraSonic_MB1013.get(); 

    //output voltage is linear with the inverse of distance, scaling factor???
    m_currDistanceInfrared2Y = m_Infrared_2Y.get();
    m_currDistanceInfrared0A = m_Infrared_0A.get();
    
    //send to the network table
    updateNetworkTable();
  }

  private void updateNetworkTable()
  {
    distanceMB1013Entry.setDouble(m_currDistanceMB1013CM);
    distanceMB1043Entry.setDouble(m_currDistanceMB1043CM);
    distanceInfrared2YEntry.setDouble(m_currDistanceInfrared2Y);
    distanceInfrared0AEntry.setDouble(m_currDistanceInfrared0A);
  }

  public double getIndividualDistance( int portNumber )
  {
    double measuredDist = 0.0;

    switch( portNumber )
    {
      case Config.MINIROBOT_MB1013_ANALOG_PORT:
        measuredDist = m_currDistanceMB1013CM;
        break;
      case Config.MINIROBOT_MB1043_ANALOG_PORT:
        measuredDist = m_currDistanceMB1043CM;
        break;
      case Config.MINIROBOT_2Y0A02_ANALOG_PORT:
        measuredDist = m_currDistanceInfrared2Y;
        break;
      case Config.MINIROBOT_0A41SK_ANALOG_PORT:
        measuredDist = m_currDistanceInfrared0A;
        break;
      default:
        logger.severe(String.format("getIndividualDistance port number %d is invalid", portNumber));
        break;
    }

    return measuredDist;
  }

  public double getCombinedDistance()
  {
    //@todo:
    //combine/process all the distance information and provide a final result
    return 0.0;
  }

  private AnalogPotentiometer getIndividualAnalogPotentiometer( int portNumber )
  {
    switch( portNumber )
    {
      case Config.MINIROBOT_MB1013_ANALOG_PORT:
        return m_ultraSonic_MB1013;
      case Config.MINIROBOT_MB1043_ANALOG_PORT:
        return m_ultraSonic_MB1043;
      case Config.MINIROBOT_2Y0A02_ANALOG_PORT:
        return m_Infrared_2Y;
      case Config.MINIROBOT_0A41SK_ANALOG_PORT:
        return m_Infrared_0A;
      default:
        logger.severe(String.format("getIndividualAnalogInput port number %d is invalid", portNumber));
        return null;
    }

  }

}
