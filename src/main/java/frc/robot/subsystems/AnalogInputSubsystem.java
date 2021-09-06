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

    m_currDistanceInfrared2Y = m_Infrared_2Y.get();
    m_currDistanceInfrared0A = m_Infrared_0A.get();
    
    //send to the network table
    updateNetworkTable();
  }

  public void updateNetworkTable()
  {

    distanceMB1013Entry.setDouble(m_currDistanceMB1013CM);
    distanceMB1043Entry.setDouble(m_currDistanceMB1043CM);
    distanceInfrared2YEntry.setDouble(0.0);
    distanceInfrared0AEntry.setDouble(0.0);

  }
}
