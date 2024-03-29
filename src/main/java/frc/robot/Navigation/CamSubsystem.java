package frc.robot.Navigation;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.pathfinding.UDPSubSystem;
import frc.robot.pathfinding.TagDataFile;
import frc.robot.pathfinding.TagHandler;
import frc.robot.pathfinding.TagDataFile.TagData;
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

   NavigationSubsystem navSub;

  public CamSubsystem(NavigationSubsystem navSub) {
    this.navSub = navSub;
  }

  private TagData[] apriltag = new TagData[30];

  // reads out what the epic camera saw
  @Override
  public void periodic() {
    String s;
    /*s = TagsSubsystem.getLastPacket();
    if (s != null) {
      TagData tagData = TagHandler.parseTagData(s);
      if (tagData != null) {
        System.out.println("First: " + tagData.apriltag + " " + tagData.x + " " + tagData.y + " " + tagData.z);
        this.apriltag[Integer.parseInt(tagData.apriltag)] = tagData;
        //Shaky math on coordinate prediction
        double x = 0.0254 * aprilTagsCoordinates[Integer.parseInt(tagData.apriltag)][0] + tagData.x;
        double z = 0.0254 * aprilTagsCoordinates[Integer.parseInt(tagData.apriltag)][2] + tagData.z;
        Transform2d trans = new Transform2d(-(navSub.x - x), -(navSub.y - z), null);
        navSub.pose.plus(trans);
      }
    }*/
    
  }

  public TagData getAprilTag(int id) {
      return apriltag[id];
  }
}