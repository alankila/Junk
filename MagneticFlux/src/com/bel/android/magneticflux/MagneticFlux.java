package com.bel.android.magneticflux;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

class SensorGraph implements SensorEventListener {
	private final String[] accuracyStrings = new String[] {
			"> 8", "4-8", "2-4", "< 2"
	};

	private final MagneticSurface xy, xz, yz;
	private final TextView feedback;
	
	private String accuracy;
	private float minX, minY, minZ;
	private float maxX, maxY, maxZ;

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

	public void reset() {
		xy.reset();
		xz.reset();
		yz.reset();
		
		minX = minY = minZ =  9999;
		maxX = maxY = maxZ = -9999;
	}
};

public class MagneticFlux extends Activity {
	//private static final String TAG = MagneticFlux.class.getSimpleName();
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