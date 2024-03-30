package frc.robot.pathfinding;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TagSubsystem extends SubsystemBase {

    // PORT
    private final int PORT = 15200;
    DatagramChannel channel;

    // ALLOCATE BYTES FOR RECEIVING
    ByteBuffer buffer = ByteBuffer.allocate(200);

    // RECEIVE PACKETS (TRUE WHEN SOCKET IS INIT)
    private boolean isEnabled = false;
    private static String lastPacket;
    private TagHandler tagHandler;

    public TagSubsystem() {
        try {
            InetSocketAddress address = new InetSocketAddress(PORT);
            this.channel = DatagramChannel.open().bind(address);
            this.isEnabled = true;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        if (isEnabled) {
            receivePacket();
        }
    }

    private void receivePacket() {
        try {
            while (channel.receive(buffer) != null) {
                buffer.flip();
                String rawText = new String(buffer.array(), buffer.arrayOffset(),
                        buffer.remaining());

                this.lastPacket=rawText;
                // tagHandler.handleRawPacket(rawText);
                System.out.println(rawText);
                buffer.clear();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static String getLastPacket() {
        return lastPacket;
    }

}
