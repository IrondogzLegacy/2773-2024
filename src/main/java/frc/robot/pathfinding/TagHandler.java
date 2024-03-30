package frc.robot.pathfinding;

import frc.robot.pathfinding.TagDataFile.TagData;

public class TagHandler {
    private TagDataFile tagDataFile;

    public TagHandler() {
        
    }

    public TagData handleRawPacket(String rawText) {
        TagData tagData = tagDataFile.parseTagData(rawText);
            if (tagData != null) {
                System.out.println("First: " + tagData.aprilTagID + " " + tagData.x + " " + tagData.y + " " + tagData.z);
            }
            return tagData;
    }
}
