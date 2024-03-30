package frc.robot.pathfinding;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Navigation.NavigationSubsystem;
import frc.robot.Navigation.OdometrySubsystem;
import frc.robot.pathfinding.TagDataFile.TagData;

public class TagSubsystem extends SubsystemBase {

    // PORT
    private final int PORT = 15200;
    DatagramChannel channel;

    // ALLOCATE BYTES FOR RECEIVING
    ByteBuffer buffer = ByteBuffer.allocate(200);

    // RECEIVE PACKETS (TRUE WHEN SOCKET IS INIT)
    private boolean isEnabled = false;
    private String lastInput;
    private TagHandler tagHandler;

    double[][] aprilTagCoordinate = {
        {593.68,  9.68,  53.38, 120},
        {637.21,  34.79, 53.38, 120},
        {652.73, 196.17, 57.13, 180},
        {652.73, 218.42, 57.13, 180},
        {578.77, 323.00, 53.38, 270},
        {  72.5, 323.00, 53.38, 270},
        { -1.50, 218.42, 57.13, 0  },
        { -1.50, 196.17, 57.13, 0  },
        { 14.02,  34.79, 53.3 , 60 },
        { 57.54,   9.68, 53.3 , 60 },
        {468.69, 146.19, 52.00, 300},
        {468.69, 177.10, 52.0 , 60 },
        {441.74, 161.62, 52.00, 180},
        {209.48, 161.62, 52.00, 0  },
        {182.73, 177.10, 52.00, 120},
        {182.73, 146.19, 52.00, 240}};

    OdometrySubsystem odomSub;

    public TagSubsystem(OdometrySubsystem odomSub) {
        // try {
        //     InetSocketAddress address = new InetSocketAddress(PORT);
        //     this.channel = DatagramChannel.open().bind(address);
        //     this.isEnabled = true;
        // } catch (IOException e) {
        //     e.printStackTrace();
        // }
        // this.odomSub = odomSub;
    }

    @Override
    public void periodic() {
        if (isEnabled) {
            receivePacket();
        }
    }

    private void receivePacket() {
        // try {
        //     while (channel.receive(buffer) != null) {
        //         buffer.flip();
        //         String rawText = new String(buffer.array(), buffer.arrayOffset(),
        //                 buffer.remaining());

        //         this.lastInput=rawText;
        //         TagData data = tagHandler.handleRawPacket(rawText);
        //         updateOdometry(data);
        //         System.out.println(rawText);
        //         buffer.clear();

        //     }
        // } catch (IOException e) {
        //     e.printStackTrace();
        // }
    }

    public String getLastPacket() {
        return lastInput;
    }

    private void updateOdometry(TagData data) {
        // double distance = Math.sqrt(data.x * data.x + data.z * data.z) * 0.0254;
        // double angle = -data.alpha + Math.toRadians(aprilTagCoordinate[data.aprilTagID][3]);
        // double processedX = Math.cos(angle) * distance;
        // double processedY = Math.sin(angle) * distance;
        // double robotX = aprilTagCoordinate[data.aprilTagID][0] + processedX;
        // double robotY = aprilTagCoordinate[data.aprilTagID][1] + processedY;
        // odomSub.setPosition(robotX, robotY);

        // Transform2d trans = new Transform2d(robotX, robotY, new Rotation2d());
        // odomSub.pose.plus(trans);
    }

}
