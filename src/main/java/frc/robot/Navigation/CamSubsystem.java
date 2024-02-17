package frc.robot.Navigation;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.pathfinding.UDPSubSystem;

public class CamSubsystem extends SubsystemBase {
  private final SerialPort serialPort = new SerialPort(115200, SerialPort.Port.kUSB);
  /*
   * Parses April Tag Data
   * 
   * Example April Tag data:
   * Example simplified April Tag data:
   * First: 1 0.328023 -0.520075 1.085208
   * More data:
   * First: 1 0.328262 -0.520233 1.085545
   */

  private TagData[] apriltag = new TagData[30];


  // reads out what the epic camera saw
  @Override
  public void periodic() {
    String s = null;
    if (s != null) {
      TagData tagData = parseTagData(s);
      if (tagData != null) {
        System.out.println("First: " + tagData.apriltag + " " + tagData.x + " " + tagData.y + " " + tagData.z);
        // If there is data, then the data will be printed
          this.apriltag[Integer.parseInt(tagData.apriltag)] = tagData;
          // Used for the data used in TurnToTagCommand.java

        
      }
    }

    // This method will be called once per scheduler run
  }
}