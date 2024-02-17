package frc.robot.pathfinding;

public class TagData {
    String apriltag;
    double x; // How far right or left (I think)
    double y; // How high or low the april tag is
    double z; // How far away (I think)
    double alpha;
}

/*
 * S = All data
 * Tokens splits data based on ";",
 * ids splits tokens based on ":"
 * ids[0] = "TAG_FOUND", when april tag is found.
 * ids[1] = the april tag ID, example: 1, 5, 16.
 * xyz is tokens[2], it is the three floats at the end of String s
 * When an april tag is found, a string will print with the id and xyz.
 */
/*private static TagData parseTagData(String s) {
    String[] tokens = s.split(";");
    String[] ids = tokens[0].split(": ");
    if (!ids[0].equals("TAG_FOUND") || tokens.length < 4) {
        return null;
    }

    String apriltag = ids[1];

    String Group1 = tokens[2];

    String[] Num = Group1.split(",");

    double XNum = Double.parseDouble(Num[0]);

    double YNum = Double.parseDouble(Num[1]);

    double ZNum = Double.parseDouble(Num[2]);

    String TagMatrix = tokens[1];

    String[] MatrixNum = TagMatrix.split(",");

    double sinAlpha = Double.parseDouble(MatrixNum[0]);
    double minusCosAlpha = Double.parseDouble(MatrixNum[2]);

    TagData data = new TagData();
    data.x = XNum;
    data.y = YNum;
    data.z = ZNum;
    data.alpha = Math.atan2(sinAlpha,minusCosAlpha);
    data.apriltag = apriltag;
    return data;
}*/
