/*
T�RKAY B�L�YOR turkaybiliyor@hotmail.com
 */
package com.rfcomm;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;

public class BluetoothService {
    // Constants that indicate the current connection state
    public static final int STATE_NONE = 0;       // we're doing nothing
    public static final int STATE_LISTEN = 1;     // now listening for incoming connections
    public static final int STATE_CONNECTING = 2; // now initiating an outgoing connection
    public static final int STATE_CONNECTED = 3;  // now connected to a remote device

    //INSECURE	"8ce255c0-200a-11e0-ac64-0800200c9a66"
    //SECURE	"fa87c0d0-afac-11de-8a39-0800200c9a66"
    //SPP		"0001101-0000-1000-8000-00805F9B34FB"
    public static final String PROTOCOL_SCHEME_RFCOMM = "RfComm Service";
    private static final UUID SERVER_UUID = UUID.fromString("0001101-0000-1000-8000-00805F9B34FB");

    // Debugging
    private static final String TAG = "BluetoothService";
    private static final boolean D = true;

    // Member fields
    private final BluetoothAdapter mAdapter;
    private final Handler mBTHandler;
    private AcceptThread mAcceptThread;
    private int mState;
    private InputStream mmInStream = null;
    private OutputStream mmOutStream = null;

    public BluetoothService(Context context, Handler handler) {
        mAdapter = BluetoothAdapter.getDefaultAdapter();
        mState = STATE_NONE;
        mBTHandler = handler;
    }

    public synchronized int getState() {
        return mState;
    }

    private synchronized void setState(int state) {
        if (D) Log.d(TAG, "setState() " + mState + " -> " + state);
        mState = state;
        // Give the new state to the Handler so the UI Activity can update
        mBTHandler.obtainMessage(MainActivity.MESSAGE_STATE_CHANGE, state, -1).sendToTarget();
    }

    public synchronized void start() {
        if (D) Log.d(TAG, "start");

        // Start the thread to listen on a BluetoothServerSocket
        if (mAcceptThread == null) {

            mAcceptThread = new AcceptThread();
            mAcceptThread.start();
            setState(STATE_LISTEN);
        }
    }

    public synchronized void stop() {
        if (D) Log.d(TAG, "stop");

        if (mAcceptThread != null) {
            mAcceptThread.cancel();
            mAcceptThread = null;
        }
        setState(STATE_NONE);
    }

    public void sendCmd(byte[] out) {
        // Create temporary object
        AcceptThread r;
        // Synchronize a copy of the ConnectedThread
        synchronized (this) {
            if (mState != STATE_CONNECTED) return;
            r = mAcceptThread;
        }
        // Perform the write unsynchronized
        r.sendCmd(out);
    }

    private void connectionLost() {
        // Send a failure message back to the Activity
        Message msg = mBTHandler.obtainMessage(MainActivity.MESSAGE_TOAST);
        Bundle bundle = new Bundle();
        bundle.putString(MainActivity.TOAST, "Bt device connection was lost");
        msg.setData(bundle);
        mBTHandler.sendMessage(msg);

        BluetoothService.this.stop();

        // Start the service over to restart listening mode
        BluetoothService.this.start();
    }

    private class AcceptThread extends Thread {
        // The local server socket
        private final BluetoothServerSocket mmServerSocket;
        BluetoothSocket socket = null;

        public AcceptThread() {
            BluetoothServerSocket tmp = null;

            // Create a new listening server socket
            try
            {
                tmp = mAdapter.listenUsingRfcommWithServiceRecord(PROTOCOL_SCHEME_RFCOMM,SERVER_UUID);
            }
            catch (IOException e) {
                Log.e(TAG, "Socket listen() failed", e);
            }
            mmServerSocket = tmp;
        }

        public void run() {

                while (mState != STATE_CONNECTED)
                {
                    try
                    {
                        socket = mmServerSocket.accept();
                        // If a connection was accepted
                        if (socket != null) {

                            Message msg = mBTHandler.obtainMessage(MainActivity.MESSAGE_DEVICE_NAME);
                            Bundle bundle = new Bundle();
                            bundle.putString(MainActivity.DEVICE_NAME, socket.getRemoteDevice().getName());
                            msg.setData(bundle);
                            mBTHandler.sendMessage(msg);

                            mmInStream = socket.getInputStream();
                            mmOutStream = socket.getOutputStream();
                            setState(STATE_CONNECTED);
                        }

                    } catch (IOException e) {
                        Log.e(TAG, "Socket accept() failed", e);
                    break;
                    }
                }

            int bytesRead = 0;
            final byte[] bytes = new byte[2048];

            while (mState == STATE_CONNECTED)
            {
                try
                {
                    bytesRead = mmInStream.read(bytes);

                    StringBuilder b = new StringBuilder();
                    for (int i = 0; i < bytesRead; i++) {
                        b.append((char) (bytes[i]));
                    }
                    String msg = b.toString();
                    mBTHandler.obtainMessage(MainActivity.MESSAGE_READ, bytesRead, -1, msg).sendToTarget();

                } catch (IOException e) {
                    Log.e(TAG, "", e);
                    connectionLost();
                }
            }
        }

        public void sendCmd(byte[] msgBuffer) {
            try {
                mmOutStream.write(msgBuffer);
                mmOutStream.flush();
                // Share the sent message back to the UI Activity
                //mBTHandler.obtainMessage(MainActivity.MESSAGE_WRITE, -1, -1, msgBuffer).sendToTarget();

            } catch (IOException e) {
                e.printStackTrace();
                Log.e(TAG, "Exception during write", e);
            }
        }

        public void cancel() {

            try {
                mmServerSocket.close();
                setState(STATE_NONE);
            } catch (IOException e) {
                Log.e(TAG, "Socket close() of server failed", e);
            }
        }
    }
}
