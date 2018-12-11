package org.firstinspires.ftc.teamcode.systems.logging;

import android.app.Application;
import android.content.pm.ApplicationInfo;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.Socket;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class RobotHubLoggingClient implements Runnable
{
    Socket loggingSocket;
    DataOutputStream loggingOutput;
    Queue<LoggingEntry> loggingEntryQueue;
    Gson gson;
    volatile boolean isRunning;

    public RobotHubLoggingClient() {
        isRunning = true;
        loggingEntryQueue = new LinkedList<>();
        gson = new Gson();
    }

    @Override
    public void run()
    {
        try
        {
            loggingSocket = new Socket(new InetSocketAddress(7001).getAddress(), 7001);
            loggingOutput = new DataOutputStream(loggingSocket.getOutputStream());
            while (isRunning) {
               if (!loggingEntryQueue.isEmpty()) {
                    this.sendEntry();
               }
            }
            loggingSocket.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    public void addLoggingEntry(LoggingEntry entry) {
        loggingEntryQueue.add(entry);
    }

    private synchronized void sendEntry() throws IOException
    {
        String message = gson.toJson(loggingEntryQueue.remove());
        loggingOutput.writeBytes(message);
    }
}
