package frc.robot.map;

public interface IMap {

    // RED IS LEFT SIDE
    // BLUE IS RIGHT SIDE
    // SEE DIAGRAM BELOW FOR REFERENCE ON RELATIVITY OF COORDINATE POINTS

    /*

    (0,y)                               (x,y)
      * _________________________________ *
       /                |                \
      /                 |                 \
      |                 |                 |
      |                 |                 |
      |_________________|_________________|
     (0,0)                              (x,0)
     */

    //// CORNERS

    // B - bottom
    // T - top
    // L - left
    // R - right

    double[] getBRCorner();
    double[] getTRCorner();
    double[] getBLCorner();
    double[] getTLCorner();

    //// OBJECTS

    /// COLOR

    // R - red (left side)
    // B - blue (right side)

    /// FACES

    // FACE

    // F - front
    // B - bottom
    // T - top
    // L - left
    // R - right

    /// WHERE ON FACE

    // M - middle of face
    // Z - left of face
    // X - right of face

    // SOURCES
    double[] getSourceRFM();

    // Red speaker - middle of front face
    double[] getSpeakerRFM();

}