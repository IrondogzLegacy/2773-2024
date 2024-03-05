package frc.robot.Navigation;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.pathfinding.UDPSubSystem;
import frc.robot.pathfinding.TagData;
import frc.robot.pathfinding.TagHandler;
import frc.robot.pathfinding.TagsSubsystem;

public class CamSubsystem extends SubsystemBase {

  /*
   * Parses April Tag Data
   * 
   * Example April Tag data:
   * Example simplified April Tag data:
   * First: 1 0.328023 -0.520075 1.085208
   * More data:
   * First: 1 0.328262 -0.520233 1.085545
   */

   double[][] aprilTagsCoordinates = {{593.68,   9.68, 53.38},
                                     {637.21,  34.79, 53.38},
                                     {652.73, 196.17, 57.13},
                                     {652.73, 218.42, 57.13},
                                     {578.77, 323.00, 53.38},
                                     {  72.5, 323.00, 53.38},
                                     { -1.50, 218.42, 57.13},
                                     { -1.50, 196.17, 57.13},
                                     { 14.02,  34.79, 53.3 },
                                     { 57.54,   9.68, 53.3 },
                                     {468.69, 146.19, 52.00},
                                     {468.69, 177.10, 52.0 },
                                     {441.74, 161.62, 52.00},
                                     {209.48, 161.62, 52.00},
                                     {182.73, 177.10, 52.00},
                                     {182.73, 146.19, 52.00}};


   NavigationSubsystem navSub;

  public CamSubsystem(NavigationSubsystem navSub) {
    this.navSub = navSub;
  }

  private TagData[] apriltag = new TagData[30];


  // reads out what the epic camera saw
  @Override
  public void periodic() {
    String s/* = null*/;
    s = TagsSubsystem.getLastPacket();
    if (s != null) {
      TagData tagData = TagHandler.parseTagData(s);
      if (tagData != null) {
        System.out.println("First: " + tagData.apriltag + " " + tagData.x + " " + tagData.y + " " + tagData.z);
        this.apriltag[Integer.parseInt(tagData.apriltag)] = tagData;
        //Shaky math on coordinate prediction
        double x = aprilTagsCoordinates[Integer.parseInt(tagData.apriltag)][0] + tagData.x;
        double z = aprilTagsCoordinates[Integer.parseInt(tagData.apriltag)][2] + tagData.z;
        Transform2d trans = new Transform2d(-(navSub.x - x), -(navSub.y - z), null);
        navSub.pose.plus(trans);
      }
    }
    
  }

  public TagData getAprilTag(int id) {
      return apriltag[id];
  }
}