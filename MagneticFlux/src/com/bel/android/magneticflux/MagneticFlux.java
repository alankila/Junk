package com.bel.android.magneticflux;

import java.util.ArrayList;
import java.util.List;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

class Calibrator {
	private static final String TAG = Calibrator.class.getSimpleName();
	private final float[][] knownPoints = new float[32][3];

	private int classify(float v) {
		if (v < -0.5) {
			return 0;
		}
		if (v < 0) {
			return 1;
		}
		if (v < 0.5) {
			return 2;
		}
		return 3;
	}

	/**
	 * Store a bunch of vectors based on their direction.
	 * 
	 * @param x
	 * @param y
	 * @param z
	 */
	public void record(float x, float y, float z) {
		float len = (float) Math.sqrt(x * x + y * y + z * z);
		float xn = x / len;
		float yn = y / len;
		int idx = classify(xn) << 3 | classify(yn) << 1 | classify(z) >> 1;

		knownPoints[idx] = new float[] { x, y, z };
	}
	
	/**
	 * System to be solved is basically:
	 * 
	 * (x - x0)^2/(1+sx)^2 + (y - y0)^2/(1+sy)^2 + (z - z0)^2/(1+sz)^2 = R^2
	 * 
	 * where x, y, z are measurement facts, and sx, sy, sz, R are found via helper
	 * variables.
	 * 
	 * @return 6 calibration parameters (x0, y0, z0, 1, 1/(1+sy), 1/(1+sz))
	 */
	public float[] extract() {
		List<float[]> aList = new ArrayList<float[]>();
		List<float[]> bList = new ArrayList<float[]>();
		/* Use the points with data to fill in stuff. */
		for (float[] f : knownPoints) {
			if (f[0] != 0 || f[1] != 0 || f[2] != 0) {
				bList.add(new float[] { -f[0] * f[0] });
				aList.add(new float[] {
						-2 * f[0],
						f[1] * f[1],
						-2 * f[1],
						f[2] * f[2],
						-2 * f[2],
						1,
				});
			}
		}
		
		Log.i(TAG, "Found " + aList.size() + " points to calibrate with");
		
		Matrix a = new Matrix(aList.toArray(new float[0][0]));
		Matrix b = new Matrix(bList.toArray(new float[0][0]));
		Matrix x = MatrixUtil.leastSquares(a, b);		
		Log.i(TAG, "Raw results: " + x);		
		float[] xv = x.getColumn(0);
		
		/* The result vector is composed of terms:
		 * 
		 * x0, k2, k2 * y0, k3, k3 * z0, k4
		 * 
		 * and k2 = (1+sx)^2 / (1+sy)^2, k3 = (1+sx)^2 / (1+sz)^2.
		 */
		return new float[] {
				xv[0],
				xv[2] / xv[1],
				xv[4] / xv[3],
				1,
				(float) Math.sqrt(xv[1]),
				(float) Math.sqrt(xv[3]),
		};
	}
}

class SensorGraph implements SensorEventListener {
	private final String[] accuracyStrings = new String[] {
			"> 8", "4-8", "2-4", "< 2"
	};

	private final MagneticSurface xy, xz, yz;
	private final TextView feedback;
	
	private String accuracy;
	private float minX, minY, minZ;
	private float maxX, maxY, maxZ;

	private Calibrator calibrator;
	
	protected SensorGraph(MagneticSurface xy, MagneticSurface xz, MagneticSurface yz, TextView fb) {
		this.xy = xy;
		this.xz = xz;
		this.yz = yz;
		feedback = fb;
	}
	
	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		this.accuracy = accuracyStrings[accuracy];
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		float x = event.values[0];
		float y = event.values[1];
		float z = event.values[2];
		
		if (calibrator != null) {
			calibrator.record(x, y, z);
		}
		
		if (x < minX) {
			minX = x;
		}
		if (x > maxX) {
			maxX = x;
		}
		if (y < minY) {
			minY = y;
		}
		if (y > maxY) {
			maxY = y;
		}
		if (z < minZ) {
			minZ = z;
		}
		if (z > maxZ) {
			maxZ = z;
		}

		xy.putPixel(x, y);
		xz.putPixel(x, z);
		yz.putPixel(y, z);
		
		feedback.setText(String.format("%+4.2f ± %s at (%+4.1f %+4.1f %+4.1f)",
				Math.sqrt(x * x + y * y + z * z),
				accuracy,
				(maxX+minX) * 0.5f,
				(maxY+minY) * 0.5f,
				(maxZ+minZ) * 0.5f));
	}

	public void startCalibration() {
		calibrator = new Calibrator();
	}
	
	public float[] endCalibration() {
		float[] data = calibrator.extract();
		calibrator = null;
		return data;
	}
	
	public void reset() {
		xy.reset();
		xz.reset();
		yz.reset();
		
		minX = minY = minZ =  9999;
		maxX = maxY = maxZ = -9999;
	}
};

public class MagneticFlux extends Activity {
	private static final String TAG = MagneticFlux.class.getSimpleName();
	SensorManager sm;

	SensorGraph a, m;
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
        sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        
        MagneticSurface mxy = (MagneticSurface) findViewById(R.id.MXY);
        MagneticSurface mxz = (MagneticSurface) findViewById(R.id.MXZ);
        MagneticSurface myz = (MagneticSurface) findViewById(R.id.MYZ);
        mxy.setRange(50);
        mxz.setRange(50);
        myz.setRange(50);
        mxy.setLabel("XY");
        mxz.setLabel("XZ");
        myz.setLabel("YZ");
        TextView mfield = (TextView) findViewById(R.id.MField);
        m = new SensorGraph(mxy, mxz, myz, mfield);

        MagneticSurface axy = (MagneticSurface) findViewById(R.id.AXY);
        MagneticSurface axz = (MagneticSurface) findViewById(R.id.AXZ);
        MagneticSurface ayz = (MagneticSurface) findViewById(R.id.AYZ);
        axy.setRange(10);
        axz.setRange(10);
        ayz.setRange(10);
        axy.setLabel("XY");
        axz.setLabel("XZ");
        ayz.setLabel("YZ");
        TextView afield = (TextView) findViewById(R.id.AField);
        a = new SensorGraph(axy, axz, ayz, afield);
        
        Button reset = (Button) findViewById(R.id.Reset);
        reset.setOnClickListener(new OnClickListener() {
			@Override
			public void onClick(View v) {
				a.reset();
				m.reset();
			}
        });

        Button calibrate = (Button) findViewById(R.id.Calibrate);
        calibrate.setOnClickListener(new OnClickListener() {
        	boolean calibrating = false;
        	
			@Override
			public void onClick(View v) {
				if (! calibrating) {
					Log.i(TAG, "Starting calibration");
					a.startCalibration();
					calibrating = true;
				} else {
					Log.i(TAG, new Matrix(a.endCalibration()).toString());;
					calibrating = false;
				}
			}
        });
        
        /* XXX kludge: redraw with correct labels. */
        a.reset();
        m.reset();
    }	
    
	@Override
    protected void onResume() {
    	super.onStart();
        sm.registerListener(m, sm.getSensorList(Sensor.TYPE_MAGNETIC_FIELD).get(0), SensorManager.SENSOR_DELAY_UI);
        sm.registerListener(a, sm.getSensorList(Sensor.TYPE_ACCELEROMETER).get(0), SensorManager.SENSOR_DELAY_UI);
    }
    
    @Override
    protected void onPause() {
    	super.onStop();
    	sm.unregisterListener(m);
    	sm.unregisterListener(a);
    }
}