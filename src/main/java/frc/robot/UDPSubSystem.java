package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;

public class UDPSubSystem extends SubsystemBase {

    // PORT
    private final int PORT = 15200;
    // UDP Socket
    private DatagramSocket socket;
    // ALLOCATE BYTES FOR RECEIVING
    private final byte[] byteAllocation = new byte[200];
    // RECEIVE PACKETS (TRUE WHEN SOCKET IS INIT)
    private boolean isEnabled = false;



    public UDPSubSystem() {
        try {
            this.socket=new DatagramSocket(this.PORT);
            this.isEnabled=true;
        } catch (IOException e) {
            // womp womp
        }
    }

    // PACKET PLACEHOLDER
    DatagramPacket UDPPacket = new DatagramPacket(byteAllocation,
            byteAllocation.length);

    @Override
    public void periodic() {
        if(isEnabled)
            receivePacket();
    }

    private void receivePacket() {
        try {
            socket.receive(UDPPacket);
            String rawText = new String(UDPPacket.getData(), 0,
                    UDPPacket.getLength() );
            // TODO: implement rawText
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
