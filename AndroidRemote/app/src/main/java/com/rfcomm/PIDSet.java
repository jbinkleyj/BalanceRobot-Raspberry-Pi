package com.rfcomm;

import android.content.Context;
import android.graphics.Color;
import android.view.Gravity;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup.LayoutParams;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;

import org.eazegraph.lib.charts.ValueLineChart;
import org.eazegraph.lib.models.ValueLinePoint;
import org.eazegraph.lib.models.ValueLineSeries;

import java.util.ArrayList;

public class PIDSet {


	static ValueLineChart mCubicValueLineChart;
	static ArrayList<Float> decibelStack;
	static int mStackSize = 100;
	static boolean is_Started = false;

	public static void showPIDSetDialog(Context context, View parent) {
		final String TAG = "BalanceRobotControl";
		final TextView textView_KP;
		final TextView textView_KI;
		final TextView textView_KD;
		final TextView textView_VS;
		final TextView textView_KM;


		final SeekBar seekBar_KP;
		final SeekBar seekBar_KI;
		final SeekBar seekBar_KD;
		final SeekBar seekBar_VS;
		final SeekBar seekBar_KM;

		View paramView = View.inflate(context, R.layout.param_dlg, null);
		LinearLayout contentView = paramView
				.findViewById(R.id.param_dlg_content_view);
		View algoLineParamView = View
				.inflate(context, R.layout.pidset, null);
		mCubicValueLineChart = algoLineParamView.findViewById(R.id.cubiclinechart);
		mCubicValueLineChart.setBackgroundColor(Color.argb(255,224,224,224));

		resetStack();

		is_Started = true;

	    //final EditText algoLineEdit = (EditText) algoLineParamView
	    //.findViewById(R.id.algo_line_edit);
		//algoLineEdit.setText(component.getKAngle() + "");
		contentView.addView(algoLineParamView);
		Button okBtn = paramView.findViewById(R.id.param_dlg_ok);
		Button canelBtn = paramView.findViewById(R.id.param_dlg_canel);
		final PopupWindow pinParamDialog = new PopupWindow(paramView,
				LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT, true);
		
		textView_KP = paramView.findViewById(R.id.textView_KP);
		textView_KI = paramView.findViewById(R.id.textView_KI);
		textView_KD = paramView.findViewById(R.id.textView_KD);
		textView_VS = paramView.findViewById(R.id.textView_VS);
		textView_KM = paramView.findViewById(R.id.textView_KM);

		int currentKM = (int)MainActivity.getKM();

		seekBar_KP = paramView.findViewById(R.id.seekBar_KP);
		seekBar_KP.setMax(100);
		seekBar_KP.setProgress((int)(MainActivity.getKP() / currentKM * 10));

		seekBar_KI = paramView.findViewById(R.id.seekBar_KI);
		seekBar_KI.setMax(100);
		seekBar_KI.setProgress((int)(MainActivity.getKI() / currentKM * 10));

		seekBar_KD = paramView.findViewById(R.id.seekBar_KD);
		seekBar_KD.setMax(100);
		seekBar_KD.setProgress((int)(MainActivity.getKD() / currentKM * 10));

		seekBar_VS = paramView.findViewById(R.id.seekBar_VS);
		seekBar_VS.setMax(100);
		seekBar_VS.setProgress((int)MainActivity.getVS());

		seekBar_KM= paramView.findViewById(R.id.seekBar_KM);
		seekBar_KM.setMax(100);
		seekBar_KM.setProgress(currentKM);


		float KP =(float) seekBar_KP.getProgress();
		String sKP = Float.toString(KP);
		textView_KP.setText("KP = " + sKP);

		float KI =(float) seekBar_KI.getProgress();
		String sKI = Float.toString(KI);
		textView_KI.setText("KI = " + sKI);

		float KD =(float) seekBar_KD.getProgress() / 10;
		String sKD = Float.toString(KD);
		textView_KD.setText("KD = " + sKD);

		float VS =(float) seekBar_VS.getProgress();
		if(VS == 0)VS = 1;
		String sVS = Float.toString(VS);
		textView_VS.setText("VS = " + sVS);

		float KM =(float) seekBar_KM.getProgress() / 10;
		if(KM == 0)KM = 1;
		String sKM = Float.toString(KM);
		textView_KM.setText("KM = " + sKM);


		OnSeekBarChangeListener seekBar_KP_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
                                          boolean fromUser) {
				float KP =(float) seekBar_KP.getProgress();
				MainActivity.setKP(KP);
				String sKP = Float.toString(KP);
				textView_KP.setText("KP = " + sKP);
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

		seekBar_KP.setOnSeekBarChangeListener(seekBar_KP_Listener);


		OnSeekBarChangeListener seekBar_KI_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
                                          boolean fromUser) {
				float KI =(float) seekBar_KI.getProgress();
				MainActivity.setKI(KI);
				String sKI = Float.toString(KI);
				textView_KI.setText("KI = " + sKI);
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

		seekBar_KI.setOnSeekBarChangeListener(seekBar_KI_Listener);

		OnSeekBarChangeListener seekBar_KD_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
                                          boolean fromUser) {
				float KD =(float) seekBar_KD.getProgress() / 10;
				MainActivity.setKD(KD);
				String sKD = Float.toString(KD);
				textView_KD.setText("KD = " + sKD);
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

		seekBar_KD.setOnSeekBarChangeListener(seekBar_KD_Listener);

		OnSeekBarChangeListener seekBar_VS_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
										  boolean fromUser) {
				float VS =(float) seekBar_VS.getProgress();
				if(VS == 0)VS = 1;
				MainActivity.setVS(VS);
				String sVS = Float.toString(VS);
				textView_VS.setText("VS = " + sVS);
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

		seekBar_VS.setOnSeekBarChangeListener(seekBar_VS_Listener);

		OnSeekBarChangeListener seekBar_KM_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
										  boolean fromUser) {
				float KM =(float) seekBar_KM.getProgress() / 10;
				if(KM == 0)KM = 1;
				MainActivity.setKM(KM);
				String sKM = Float.toString(KM);
				textView_KM.setText("KM = " + sKM);
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

		seekBar_KM.setOnSeekBarChangeListener(seekBar_KM_Listener);


		okBtn.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View v) {
				pinParamDialog.dismiss();
				//component.setScale(Float.parseFloat(algoLineEdit.getText()
						//.toString()));
				MainActivity.setVisible();
			}
		});
		
		canelBtn.setOnClickListener(new OnClickListener() {
			
			@Override
			public void onClick(View v) {
				pinParamDialog.dismiss();
				MainActivity.setVisible();
				//addValue(Math.sin(graph2LastXValue*0.5) * 20*(Math.random()*10+1));
			}
		});
		pinParamDialog.showAtLocation(parent, Gravity.CENTER, 0, 0);
	}

	static void resetStack() {
		decibelStack = new ArrayList<>();
		for (int i =0; i < mStackSize; i++) {
			decibelStack.add(0f);
		}
	}

	static synchronized void addValue(float decibel) {

		if(is_Started)
		{
			if (decibelStack.size() > mStackSize) {
				decibelStack.remove(0);
			}
			decibelStack.add(decibelStack.size(), decibel);

			refreshGraph();
		}
	}

	static void refreshGraph() {
		mCubicValueLineChart.clearChart();
		final ValueLineSeries series = new ValueLineSeries();
		series.setColor(Color.BLUE);
		for (float i : decibelStack) {
			series.addPoint(new ValueLinePoint(String.valueOf(i), i));
		}
		mCubicValueLineChart.addSeries(series);
	}

}
