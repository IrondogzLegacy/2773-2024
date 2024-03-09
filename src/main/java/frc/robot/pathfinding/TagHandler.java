package frc.robot.pathfinding;

public class TagHandler {
    
    public TagData data = new TagData();

    public void parseTagData(String s) {
        String[] tokens = s.split(";");
        String[] ids = tokens[0].split(": ");
        if (!ids[0].equals("TAG_FOUND") || tokens.length < 4) {
            data = null;
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

        data.x = XNum;
        data.y = YNum;
        data.z = ZNum;
        data.theta = Math.atan2(sinAlpha,minusCosAlpha);
        data.aprilTagID = apriltag;

        /*
         *  cos 0  sin
         *  0   1   0
         * -sin 0 cos
         */

        //Example Data: TAG_FOUND: 1; 0.1, 0.00001, 0.1, 0.00001, 0.9999, 0.00001, -0.1, 0.0001, 0.1;  5, 6, 4;
        //                  Index   ; cos   0       sin     0        1        0    -sin    0     cos;  x  y  z;
        /* Parsing Tag Data Psuedocode
         * A - Split Tags into index 0 (id), 1 (rotation), 2 (position)
         * B - Mark all of these variables
         * C - C
         */


    }
}
