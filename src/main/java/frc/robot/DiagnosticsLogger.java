/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;
import java.util.Map;

/**
 * This class listens on a port and then sends diagnostics data to clients.
 */
public class DiagnosticsLogger {
    private Map<String, Integer> intMap;
    private Map<String, String> stringMap;
    private Map<String, Double> doubleMap;
    private ServerSocket server;
    private Socket client;
    private OutputStream clientStream;
    private Object clientLock = new Object();

    public DiagnosticsLogger() {
        intMap = new HashMap<String, Integer>();
        stringMap = new HashMap<String, String>();
        doubleMap = new HashMap<String, Double>();
    }

    public void start() {
        Thread t = new Thread(() -> {
            while (!Thread.interrupted()) {
                try {
                    server = new ServerSocket(5800);
                    
                    while (!Thread.interrupted()) {
                        Socket socket;
                        socket = server.accept();

                        synchronized (clientLock) {
                            client = socket;
                            clientStream = socket.getOutputStream();
                        }
                        
                        // Wait for client to close connection
                        while (!Thread.interrupted()) {
                            synchronized (clientLock) {
                                if (clientStream == null) {
                                    break;
                                }
                            }
                            Thread.sleep(1000);
                        }
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        t.start();
    }

    public void writeInteger(String name, int value) {
        if (!intMap.containsKey(name) || intMap.get(name) != value) {
            intMap.put(name, value);

            sendString(name, Integer.toString(value));
        }
    }

    public void writeDouble(String name, double value) {
        if (!doubleMap.containsKey(name) || doubleMap.get(name) != value) {
            doubleMap.put(name, value);

            sendString(name, Double.toString(value));
        }
    }

    public void writeString(String name, String value) {
        if (!stringMap.containsKey(name) || !stringMap.get(name).equals(value)) {
            stringMap.put(name, value);

            sendString(name, value);
        }
    }

    public void sendString(String name, String value) {
            String msg = name + ":" + value + "\n";
            //System.out.println(msg);

            try {
                if (clientStream != null && client.isConnected()) {
                    clientStream.write(msg.getBytes());
                }
            } catch (IOException e) {
                e.printStackTrace();
                clientStream = null;
            }
    }

	public void timestamp() {
        if (clientStream != null && client.isConnected()) {
            try {
                sendString("Time", Long.toString(System.currentTimeMillis()));
            } catch (Exception e) {
                System.out.println("Failed to flush diagnostic client stream:");
                e.printStackTrace();
                clientStream = null;
            }
        }
	}
}
