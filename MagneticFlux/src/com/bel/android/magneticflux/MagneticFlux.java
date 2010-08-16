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

public class MagneticFlux extends Activity {
	SensorManager sm;
	
	MagneticSurface xy, xz, yz;

	TextView field, boundX, boundY, boundZ;
	
	private int minX, maxX, minY, maxY, minZ, maxZ;
	
	private final SensorEventListener sensorListener = new SensorEventListener() {
		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {
		}

		@Override
		public void onSensorChanged(SensorEvent event) {
			int x = (int) event.values[0];
			int y = (int) event.values[1];
			int z = (int) event.values[2];
			
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
			
			field.setText(String.format("%.0f uT", Math.sqrt(x * x + y * y + z * z)));
			boundX.setText(String.format("X: %+3d - %+3d uT, center %+3d", minX, maxX, (minX + maxX) / 2));
			boundY.setText(String.format("Y: %+3d - %+3d uT, center %+3d", minY, maxY, (minY + maxY) / 2));
			boundZ.setText(String.format("Z: %+3d - %+3d uT, center %+3d", minZ, maxZ, (minZ + maxZ) / 2));
		}
		
	};
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
        sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        xy = (MagneticSurface) findViewById(R.id.XY);
        xz = (MagneticSurface) findViewById(R.id.XZ);
        yz = (MagneticSurface) findViewById(R.id.YZ);
        
        xy.setLabel("XY");
        xz.setLabel("XZ");
        yz.setLabel("YZ");
        
        field = (TextView) findViewById(R.id.Field);
        boundX = (TextView) findViewById(R.id.BoundX);
        boundY = (TextView) findViewById(R.id.BoundY);
        boundZ = (TextView) findViewById(R.id.BoundZ);
        
        Button reset = (Button) findViewById(R.id.Reset);
        reset.setOnClickListener(new OnClickListener() {
			@Override
			public void onClick(View v) {
		        sm.unregisterListener(sensorListener);
				
				xy.reset();
				xz.reset();
				yz.reset();
				
				minX = 0;
				maxX = 0;
				minY = 0;
				maxY = 0;
				minZ = 0;
				maxZ = 0;
				
		    	Sensor sensor = sm.getSensorList(Sensor.TYPE_MAGNETIC_FIELD).get(0);
		        sm.registerListener(sensorListener, sensor, SensorManager.SENSOR_DELAY_UI);
			}
        });
    }
    
    @Override
    protected void onResume() {
    	super.onResume();
    	Sensor sensor = sm.getSensorList(Sensor.TYPE_MAGNETIC_FIELD).get(0);
        sm.registerListener(sensorListener, sensor, SensorManager.SENSOR_DELAY_UI);
        xy.reset();
        xz.reset();
        yz.reset();
    }
    
    @Override
    protected void onStop() {
    	super.onStop();
    	sm.unregisterListener(sensorListener);
    }
}