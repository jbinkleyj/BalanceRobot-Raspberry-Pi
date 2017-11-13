package com.rfcomm;

import java.util.StringTokenizer;

import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.PowerManager;
import android.os.SystemClock;
import android.support.v7.app.ActionBarActivity;
import android.util.Log;
import android.view.KeyEvent;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.view.inputmethod.EditorInfo;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;


public class MainActivity extends ActionBarActivity
{
	public static final int MESSAGE_STATE_CHANGE = 1;
	public static final int MESSAGE_READ = 2;
	public static final int MESSAGE_WRITE = 3;
	public static final int MESSAGE_DEVICE_NAME = 4;
	public static final int MESSAGE_TOAST = 5;

	public static final String DEVICE_NAME = "device_name";
	private String mConnectedDeviceName = "Pi RfComm";
	public static final String TOAST = "toast";

	private final static int REQUEST_ENABLE_BT = 0;
	
	private static final String TAG = MainActivity.class.getSimpleName();
	private static BluetoothService  mBtService = null;

	/* GetDefaultAdapter */
	private BluetoothAdapter _bluetooth = BluetoothAdapter.getDefaultAdapter();


	private static Context mContext;
	private View menuView;

	byte[] commandPacket = new byte[11];
	static byte[] commandPacketBlueTooth = new byte[6];

	public static float Kp = 0;
	public static float Ki = 0;
	public static float Kd = 0;
	public static float Angle_Error = 0;

	private int speed1;
	private int speed2;

	private Button Pids;
	private Button Forward;
	private Button Back;
	private Button Left;
	private Button Right;
	private Button StartStop;
	private Button C_Plus;
	private Button C_Mines;

	static TextView textView_Speed1;
	static TextView textView_Speed2;
	static TextView  textPwmL;
	static TextView  textPwmR;
	static TextView  textSpeedL;
	static TextView  textAngle;
	static TextView  textSpeedR;

	static SeekBar seekBar_Speed1;
	static SeekBar seekBar_Speed2;

	private PowerManager.WakeLock wl;
	private Menu menu;
	private EditText mOutEditText;
	private TextView statusText;
	private Button mSendButton;
	private ListView mConversationView;
	private ArrayAdapter<String> mConversationArrayAdapter;

	int connectionState = BluetoothService.STATE_NONE;

	private final Handler mBtHandler = new Handler() {
		@Override
		public void handleMessage(Message msg) {

			switch (msg.what) {
				case MESSAGE_STATE_CHANGE:

					connectionState = msg.arg1;

					switch (connectionState) {
						case BluetoothService.STATE_CONNECTED:
							statusText.setText(getString(R.string.title_connected_to, mConnectedDeviceName));
							break;
						case BluetoothService.STATE_LISTEN:
							statusText.setText(getString(R.string.title_listening));							
							break;
						case BluetoothService.STATE_NONE:
							statusText.setText(getString(R.string.title_not_connected));
							StartStop.setText("Start");
							break;
					}
					break;
				case MESSAGE_WRITE:

					byte[] writeBuf = (byte[]) msg.obj;
					String writeMessage = new String(writeBuf);

					break;
				case MESSAGE_READ:

					String tmpmsg = msg.obj.toString();
					parseMessage(tmpmsg);

					break;

				case MESSAGE_DEVICE_NAME:

					mConnectedDeviceName = msg.getData().getString(DEVICE_NAME);
					break;

				case MESSAGE_TOAST:
					Toast.makeText(getApplicationContext(), msg.getData().getString(TOAST),
							Toast.LENGTH_SHORT).show();
					break;
			}
		}
	};


	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		setContentView(R.layout.main);

		mContext = this;

		PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
		wl = pm.newWakeLock(PowerManager.SCREEN_BRIGHT_WAKE_LOCK, "My Tag");
		wl.acquire();

		getWindow().setSoftInputMode(
				WindowManager.LayoutParams.SOFT_INPUT_STATE_ALWAYS_HIDDEN
		);

		commandPacket[0] = (byte) 0xFF;
		commandPacket[1] = (byte) 0xAA;
		commandPacket[2] = 0x00;
		commandPacket[3] = 0x00;
		commandPacket[4] = 0x00;
		commandPacket[5] = 0x00;
		commandPacket[8] = 0x00;
		commandPacket[9] = 0x00;
		commandPacket[10] = 0x02;

		commandPacketBlueTooth[0] = (byte)0xAA;

		statusText = (TextView) findViewById(R.id.text_status);
		mOutEditText = (EditText) findViewById(R.id.edit_text_out);
		mSendButton = (Button) findViewById(R.id.button_send);
		mConversationView = (ListView) findViewById(R.id.in);

		// Initialize the array adapter for the conversation thread
		mConversationArrayAdapter = new ArrayAdapter<String>(this, R.layout.message) {
			@Override
			public View getView(int position, View convertView, ViewGroup parent) {
				// Get the Item from ListView
				View view = super.getView(position, convertView, parent);

				// Initialize a TextView for ListView each Item
				TextView tv = view.findViewById(R.id.listText);

				// Set the text color of TextView (ListView Item)
				tv.setTextColor(Color.parseColor("#FFFFFF"));
				tv.setTextSize(12);

				// Generate ListView Item using TextView
				return view;
			}
		};

		mConversationView.setAdapter(mConversationArrayAdapter);

		mSendButton.setOnClickListener(new View.OnClickListener() {
			public void onClick(View v) {
				// Send a message using content of the edit text widget
				String message = mOutEditText.getText().toString();
				sendMessage(message);
			}
		});

		mOutEditText.setOnEditorActionListener(mWriteListener);

		textView_Speed1 = (TextView)findViewById(R.id.textView_Speed1);
		textView_Speed2 = (TextView)findViewById(R.id.textView_Speed2);
		textPwmL = (TextView)findViewById(R.id.textPwmL);
		textPwmR = (TextView)findViewById(R.id.textPwmR);
		textSpeedL = (TextView)findViewById(R.id.textSpeedL);
		textSpeedR = (TextView)findViewById(R.id.textSpeedR);
		textAngle = (TextView)findViewById(R.id.textAngle);

		seekBar_Speed1 = (SeekBar)findViewById(R.id.seekBar_Speed1);
		seekBar_Speed1.setMax(255);
		//seekBar_Speed1.setProgress(MakerStudioDemo.getKAngle());

		seekBar_Speed2 = (SeekBar) findViewById(R.id.seekBar_Speed2);
		seekBar_Speed2.setMax(255);

		SeekBar.OnSeekBarChangeListener seekBar_Speed1_Listener = new SeekBar.OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
										  boolean fromUser) {
				//App.setKAngle(seekBar_KAngle.getProgress());
				textView_Speed1.setText("Forward-Reverse Speed : " + seekBar_Speed1.getProgress());
				speed1 = seekBar_Speed1.getProgress();
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};
		seekBar_Speed1.setOnSeekBarChangeListener(seekBar_Speed1_Listener);

		SeekBar.OnSeekBarChangeListener seekBar_Speed2_Listener = new SeekBar.OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
										  boolean fromUser) {
				textView_Speed2.setText("Turn Speed : " + seekBar_Speed2.getProgress());
				speed2 = seekBar_Speed2.getProgress();
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};
		seekBar_Speed2.setOnSeekBarChangeListener(seekBar_Speed2_Listener);


		LayoutInflater mLayoutInflater = (LayoutInflater) getSystemService(Context.LAYOUT_INFLATER_SERVICE);
		menuView = mLayoutInflater.inflate(
				R.layout.main, null, true);


		Pids = (Button) findViewById(R.id.Pids);
		Pids.setOnClickListener(PidClickListener);

		Forward = (Button) findViewById(R.id.Forward);
		Forward.setOnTouchListener(new Button.OnTouchListener() {
			@Override
			public boolean onTouch(View v, MotionEvent event) {
				int action = event.getAction();
				switch (action) {
					case MotionEvent.ACTION_DOWN:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x01;
						commandPacketBlueTooth[3] = 0x03;
						commandPacketBlueTooth[4] = (byte)speed1;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;

					case MotionEvent.ACTION_UP:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x00;
						commandPacketBlueTooth[3] = 0x03;
						commandPacketBlueTooth[4] = 0x30;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;
				}
				return false;
			}
		});


		Back = (Button) findViewById(R.id.Back);
		Back.setOnTouchListener(new Button.OnTouchListener() {
			@Override
			public boolean onTouch(View v, MotionEvent event) {
				int action = event.getAction();
				switch (action) {
					case MotionEvent.ACTION_DOWN:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x02;
						commandPacketBlueTooth[3] = 0x00;
						commandPacketBlueTooth[4] = (byte)speed1;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;

					case MotionEvent.ACTION_UP:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x00;
						commandPacketBlueTooth[3] = 0x00;
						commandPacketBlueTooth[4] = 0x30;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;
				}
				return false;
			}
		});

		Left = (Button) findViewById(R.id.Left);
		Left.setOnTouchListener(new Button.OnTouchListener() {
			@Override
			public boolean onTouch(View v, MotionEvent event) {
				int action = event.getAction();
				switch (action) {
					case MotionEvent.ACTION_DOWN:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x03;
						commandPacketBlueTooth[3] = 0x00;
						commandPacketBlueTooth[4] = (byte)speed2;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;

					case MotionEvent.ACTION_UP:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x00;
						commandPacketBlueTooth[3] = 0x03;
						commandPacketBlueTooth[4] = 0x03;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;
				}
				return false;
			}
		});

		Right = (Button) findViewById(R.id.Right);
		Right.setOnTouchListener(new Button.OnTouchListener() {

			@Override
			public boolean onTouch(View v, MotionEvent event) {
				int action = event.getAction();
				switch (action) {
					case MotionEvent.ACTION_DOWN:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x04;
						commandPacketBlueTooth[3] = 0x00;
						commandPacketBlueTooth[4] = (byte)speed2;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;

					case MotionEvent.ACTION_UP:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x00;
						commandPacketBlueTooth[3] = 0x03;
						commandPacketBlueTooth[4] = 0x03;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;
				}
				return false;
			}
		});

		StartStop = (Button) findViewById(R.id.StartStop);
		StartStop.setText("Start");

		StartStop.setOnClickListener(new View.OnClickListener() {
			public void onClick(View v) {

				if(connectionState != BluetoothService.STATE_CONNECTED)
					return;

				if(StartStop.getText() == "Start")
				{
					commandPacketBlueTooth[1] = 0x03;
					commandPacketBlueTooth[2] = 0x07;
					commandPacketBlueTooth[3] = 0x03;
					commandPacketBlueTooth[4] = 0x03;
					commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
					if (mBtService != null) {
						mBtService.sendCmd(commandPacketBlueTooth);
						StartStop.setText("Stop");
					}

				}
				else
				{
					commandPacketBlueTooth[1] = 0x03;
					commandPacketBlueTooth[2] = 0x08;
					commandPacketBlueTooth[3] = 0x03;
					commandPacketBlueTooth[4] = 0x03;
					commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
					if (mBtService != null) {
						mBtService.sendCmd(commandPacketBlueTooth);
						StartStop.setText("Start");
					}
				}
			}
		});

		C_Plus = (Button) findViewById(R.id.C_Plus);
		C_Plus.setOnTouchListener(new Button.OnTouchListener() {

			@Override
			public boolean onTouch(View v, MotionEvent event) {
				int action = event.getAction();
				switch (action) {
					case MotionEvent.ACTION_DOWN:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x05;
						commandPacketBlueTooth[3] = 0x00;
						commandPacketBlueTooth[4] = (byte)speed2;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;
				}
				return false;
			}
		});

		C_Mines = (Button) findViewById(R.id.C_Mines);
		C_Mines.setOnTouchListener(new Button.OnTouchListener() {

			@Override
			public boolean onTouch(View v, MotionEvent event) {
				int action = event.getAction();
				switch (action) {
					case MotionEvent.ACTION_DOWN:
						commandPacketBlueTooth[1] = 0x03;
						commandPacketBlueTooth[2] = 0x06;
						commandPacketBlueTooth[3] = 0x00;
						commandPacketBlueTooth[4] = (byte)speed2;
						commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
						if (mBtService != null)
							mBtService.sendCmd(commandPacketBlueTooth);
						break;
				}
				return false;
			}
		});

		if (!_bluetooth.isEnabled()) {
			Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
			startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
			return;
		}

		mBtService = new BluetoothService(this, mBtHandler);
		mBtService.start();
	}

	private void sendMessage(String message) {

		if (mBtService != null)
		{
			if (mBtService.getState() != BluetoothService.STATE_CONNECTED) {
				return;
			}
			try {
				if (message.length() > 0) {
					byte[] send = message.getBytes();
					mBtService.sendCmd(send);
				}
			} catch (Exception e) {
			}
		}
	}


	public static byte exclusiveOr(byte[] commandPacket) {
		Byte result = null;
		return result = (byte) (commandPacket[2] ^ commandPacket[3] ^ commandPacket[4]);

	}

	private View.OnClickListener PidClickListener = new View.OnClickListener() {
		@Override
		public void onClick(View arg0) {

			if (mBtService != null) {
				textView_Speed1.setVisibility(View.INVISIBLE);
				textView_Speed2.setVisibility(View.INVISIBLE);
				seekBar_Speed1.setVisibility(View.INVISIBLE);
				seekBar_Speed2.setVisibility(View.INVISIBLE);
				PIDSet.showPIDSetDialog(mContext, menuView);
			}
		}
	};

	// The action listener for the EditText widget, to listen for the return key
	private TextView.OnEditorActionListener mWriteListener =
			new TextView.OnEditorActionListener() {
				public boolean onEditorAction(TextView view, int actionId, KeyEvent event) {
					// If the action is a key-up event on the return key, send the message
					if (actionId == EditorInfo.IME_NULL && event.getAction() == KeyEvent.ACTION_UP) {
						String message = view.getText().toString();
						sendMessage(message);
					}
					return true;
				}
			};

	public void onActivityResult(int requestCode, int resultCode, Intent data) {
		switch (requestCode) {

			case REQUEST_ENABLE_BT:

				if(mBtService == null)
				{
					mBtService = new BluetoothService(this, mBtHandler);
					mBtService.start();
				}
				break;
		}
	}

	protected void onDestroy() {
		super.onDestroy();
		mBtService.stop();
		wl.release();
	}
	protected void finalize() throws Throwable {
		super.finalize();
		mBtService.stop();
	}

	@Override
	public boolean onKeyDown(int keyCode, KeyEvent event) {
		super.onKeyDown(keyCode, event);
		if (keyCode == KeyEvent.KEYCODE_BACK) {
			exit();
		}
		return super.onKeyDown(keyCode, event);
	}

	private void exit() {
		wl.release();
		android.os.Process.killProcess(android.os.Process.myPid());
	}


	private String clearMsg(String msg) {

		String tmpmsg = msg.toString();
		tmpmsg = tmpmsg.replace("null", "");
		//tmpmsg = tmpmsg.replaceAll("\\s", ""); //removes all [ \t\n\x0B\f\r]
		tmpmsg = tmpmsg.replace("\n", "").replace("\r", "");
		tmpmsg = tmpmsg.replaceAll(">", "");

		return tmpmsg;
	}


	public void parseMessage(String msg)
	{

		String tmpmsg = clearMsg(msg);

		if(tmpmsg.contains("Data:"))
		{
			String Title = "Data";
			String Pwml = "0";
			String Pwmr = "0";
			String Angle ="0";
			String Speed_Need = "0";
			String Turn_Need = "0";
			String Speed_L = "0";
			String Speed_R = "0";
			String Temperature = "0";
			String KP = "0";
			String KI = "0";
			String KD = "0";
			String Correction = "0";
			String Error = "0";

			StringTokenizer token = new StringTokenizer(tmpmsg, ":");
			Log.e(TAG,tmpmsg);

			if(token.hasMoreTokens()) {
				Title = token.nextToken();
			}

			if(token.hasMoreTokens())
			{
				Pwml = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				Pwmr = token.nextToken();
			}
			if(token.hasMoreTokens()) {
				Angle = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				Speed_Need = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				Turn_Need = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				Speed_L = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				Speed_R = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				KP = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				KI = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				KD = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				Temperature = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				Correction = token.nextToken();
			}
			if(token.hasMoreTokens())
			{
				Error = token.nextToken();
			}


			Kp = Float.valueOf(KP.replace(",","."));
			Ki = Float.valueOf(KI.replace(",","."));
			Kd = Float.valueOf(KD.replace(",","."));
			Angle_Error = Float.valueOf(Error.replace(",","."));
			PIDSet.addValue(Angle_Error);

			//Log.e(TAG,"KP : " + Kp + "  KI : " + Ki + "  Kd : " + Kd );

			tmpmsg =  "KP : " + Kp + "  KI : " + Ki + "  KD : " + Kd + "\n"
					+ "PwmL: " + Pwml +  "  PwmR: " + Pwmr + "  SpN: : " + Speed_Need +  "  TnN: " + Turn_Need + "\n"
					+ "Temp: " + Temperature +  "  Correction: " + Correction +  "  Error: " + Error;

			textSpeedL.setText("L: " + Speed_L);
			textPwmR.setText("PR: " + Pwmr);
			textPwmL.setText("PL: " + Pwml);
			textAngle.setText("A: " + Angle);
			textSpeedR.setText("R: " + Speed_R);

		}

		mConversationArrayAdapter.add("Read:  " + tmpmsg);
	}

	public static float getKP() {
		return Kp;
	}

	public static void setKP(float KP) {

		Kp = KP;
		commandPacketBlueTooth[1] = 0x02;
		commandPacketBlueTooth[2] = 0x01;
		float ikp = KP * 100;
		commandPacketBlueTooth[3] = (byte)(ikp / 255);
		commandPacketBlueTooth[4] = (byte)(ikp % 255);
		//commandPacketBlueTooth[4] = 0x03;
		commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
		if (mBtService != null)
			mBtService.sendCmd(commandPacketBlueTooth);
	}

	public static float getKI() {
		return Ki;
	}

	public static void setKI(float KI) {

		Ki = KI;
		commandPacketBlueTooth[1] = 0x02;
		commandPacketBlueTooth[2] = 0x02;
		float iki = KI * 100;
		commandPacketBlueTooth[3] = (byte)(iki / 255);
		commandPacketBlueTooth[4] = (byte)(iki % 255);
		//commandPacketBlueTooth[4] = 0x03;
		commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
		if (mBtService != null)
			mBtService.sendCmd(commandPacketBlueTooth);
	}

	public static float getKD() {
		return Kd * 10;
	}

	public static void setKD(float KD) {

		Kd = KD;
		commandPacketBlueTooth[1] = 0x02;
		commandPacketBlueTooth[2] = 0x03;
		float ikd = KD * 100;
		commandPacketBlueTooth[3] = (byte)(ikd / 255);
		commandPacketBlueTooth[4] = (byte)(ikd % 255);
		//commandPacketBlueTooth[4] = 0x03;
		commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
		if (mBtService != null)
			mBtService.sendCmd(commandPacketBlueTooth);
	}

	public static void setVisible() {
		textView_Speed1.setVisibility(View.VISIBLE);
		textView_Speed2.setVisibility(View.VISIBLE);
		seekBar_Speed1.setVisibility(View.VISIBLE);
		seekBar_Speed2.setVisibility(View.VISIBLE);
	}
}
