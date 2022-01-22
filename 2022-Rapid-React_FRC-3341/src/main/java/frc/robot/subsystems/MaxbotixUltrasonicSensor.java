// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C;
/** Class for reporting data from Maxbotix I2CXL MaxSonar sensors. */
public class MaxbotixUltrasonicSensor extends SubsystemBase {
  
  private final I2C.Port i2cPort;
  private I2C i2cController;

  private byte inputDataArray[] = new byte[2];

  private int highByteTwosComplement;
  private int lowByteTwosComplement;

  private double reportedDistance;
  private double distance;

  private boolean isBusy = false;

  /**
   * Constructor.
   *
   * @param address The address of the sensor on the I2C bus.
   */
  public MaxbotixUltrasonicSensor(int address) {
    i2cPort = I2C.Port.kOnboard;

    // The RoboRIO uses 7 bit addressing, so the address here is 112
    i2cController = new I2C(i2cPort, address);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // An example of it's intended usage
    reportedDistance = getDistance();

    if (reportedDistance != -1.0) {
      SmartDashboard.putNumber("Range", reportedDistance);
    }

  }

  private void requestDistanceValue() {
    i2cController.write(224, 81);
  }

  private void readDistanceValue() {
    // Read two bytes of data
    i2cController.read(225, 2, inputDataArray);
    highByteTwosComplement = inputDataArray[0];
    lowByteTwosComplement = inputDataArray[1];

    // Correct the two's complement for both unsigned bytes
    if (highByteTwosComplement < 0) highByteTwosComplement += 256;
    if (lowByteTwosComplement < 0) lowByteTwosComplement += 256;
    /*
    Concatenate the highbyte (index 0) and lowbyte (index 1) 
    Concatenated bytes are in units of centimeters,
    divided by 100 to get meters

    Roughly based off of the official Arduino library
    */
    distance = (lowByteTwosComplement + highByteTwosComplement*256.0)/100.0;
  }
  /** Gets the distance in meters that the sensor reports. */
  public double getDistance() {
    if (!isBusy) {
      isBusy = true;

      Timer.delay(0.1); // For safety, if it has multiple calls
      requestDistanceValue();
      // Mfr. recommends 80 to 100 ms delay between request and read
      Timer.delay(0.1);
      readDistanceValue();

      isBusy = false;

      return distance;
    } else {
      return -1.0;
    }

  }

}
