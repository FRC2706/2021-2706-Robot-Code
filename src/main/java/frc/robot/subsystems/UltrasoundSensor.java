// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class UltrasoundSensor extends SubsystemBase {

  private NetworkTableEntry analog1043ValueEntry, analog1013ValueEntry, distance1043Entry, distance1013Entry;

  AnalogInput Analog1043 = new AnalogInput(4);
  AnalogInput Analog1013 = new AnalogInput(5);
  

  /** Creates a new UltrasoundSensor. */
  public UltrasoundSensor() {
    var table = NetworkTableInstance.getDefault().getTable("Ultrasound Sensor Data");
    analog1043ValueEntry = table.getEntry("1043 analog value");
    analog1013ValueEntry = table.getEntry("1013 analog value");
    distance1043Entry = table.getEntry("1043 distance value (m)");
    distance1013Entry = table.getEntry("1013 distance value (m)");
  }

  @Override
  public void periodic() {
    //Get the voltage readings
    double analog1043Value = Analog1043.getVoltage();
    double analog1013Value = Analog1013.getVoltage();

    //Convert the voltage to distance travelled (meters)
    double distance1043 = (analog1043Value/4.88)*5;
    double distance1013 = (analog1013Value/4.88)*5;
    
    analog1043ValueEntry.setNumber(analog1043Value);
    analog1013ValueEntry.setNumber(analog1013Value);
    distance1043Entry.setNumber(distance1043);
    distance1013Entry.setNumber(distance1013);
    // This method will be called once per scheduler run
  }
}
