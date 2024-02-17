package frc.robot.map;

public class Map implements IMap {


    @Override
    public double[] getBRCorner() {
        return new double[] {16.535D, 0D};
    }

    @Override
    public double[] getTRCorner() {
        return new double[] {16.535D, 6.096D};
    }

    @Override
    public double[] getBLCorner() {
        return new double[] {0D, 0D};
    }

    @Override
    public double[] getTLCorner() {
        return new double[] {0D, 6.096D};
    }

    @Override
    public double[] getSourceRFM() {
        return new double[] {0D, 8.5415D};
    }

    @Override
    public double[] getSpeakerRFM() {
        return new double[] {0D, 6.096D};
    }

}
