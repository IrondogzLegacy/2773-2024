package frc.robot.pathfinding;

import frc.robot.pathfinding.TagDataFile.TagData;

public class TagHandler {
    private TagDataFile tagDataFile;

    public TagData handleRawPacket(String rawText) {
        TagData tagData = tagDataFile.parseTagData(rawText);
            if (tagData != null) {
                System.out.println("First: " + tagData.apriltagID + " " + tagData.x + " " + tagData.y + " " + tagData.z);
                // If there is data, then the data will be printed
                return tagData;
            }
            return tagData;
        }



    // public static TagDataFile parseTagData(String s) {
    //     String[] tokens = s.split(";");
    //     String[] ids = tokens[0].split(": ");
    //     if (!ids[0].equals("TAG_FOUND") || tokens.length < 4) {
    //         return null;
    //     }

    //      String apriltagID = ids[1];

    //     String Group1 = tokens[2];

    //     String[] Num = Group1.split(",");

    //      double XNum = Double.parseDouble(Num[0]);

    //      double YNum = Double.parseDouble(Num[1]);

    //      double ZNum = Double.parseDouble(Num[2]);

    //     String TagMatrix = tokens[1];

    //     String[] MatrixNum = TagMatrix.split(",");

    //     double sinAlpha = Double.parseDouble(MatrixNum[0]);
    //     double minusCosAlpha = Double.parseDouble(MatrixNum[2]);

    //     TagDataFile data = new TagDataFile();
    //     data.x = XNum;
    //     data.y = YNum;
    //     data.z = ZNum;
    //     data.alpha = Math.atan2(sinAlpha,minusCosAlpha);
    //     data.apriltagID = apriltagID;
    //     return data;
    // }

}
