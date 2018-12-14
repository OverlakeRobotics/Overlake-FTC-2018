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
    boolean isRunning;

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
            loggingSocket = new Socket("192.168.49.1", 7001);
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
        LoggingEntry entry = loggingEntryQueue.remove();
        if (entry == null) {
            isRunning = false;
            loggingOutput.flush();
            loggingSocket.close();
        } else {
            String message = gson.toJson(entry) + "\n";
            loggingOutput.writeBytes(message);
            loggingOutput.flush();
        }
    }
}
