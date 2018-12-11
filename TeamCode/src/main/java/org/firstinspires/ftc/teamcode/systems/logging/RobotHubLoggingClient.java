package org.firstinspires.ftc.teamcode.systems.logging;

import android.app.Application;
import android.content.pm.ApplicationInfo;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.LinkedList;
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
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    public void addLoggingEntry(LoggingEntry entry) {
        if (loggingSocket != null && loggingSocket.isConnected()) {
            loggingEntryQueue.add(entry);
        }
    }

    private synchronized void sendEntry() throws IOException
    {
        String message = gson.toJson(loggingEntryQueue.remove()) + "\n";
        loggingOutput.writeBytes(message);
    }

    public void stop() {
        isRunning = false;
        try
        {
            loggingSocket.close();
            loggingOutput.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
