package frc.robot.pathfinding;

public class TagHandler {

    public static TagData handleRawPacket(String rawText) {
        TagData tagData = parseTagData(rawText);
            if (tagData != null) {
                System.out.println("First: " + tagData.apriltag + " " + tagData.x + " " + tagData.y + " " + tagData.z);
                // If there is data, then the data will be printed
                return tagData;
            }
            return tagData;
        }

    private static TagData parseTagData(String s) {
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
    }

}
