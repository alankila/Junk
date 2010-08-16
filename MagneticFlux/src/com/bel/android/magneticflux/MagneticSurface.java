package com.bel.android.magneticflux;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Bitmap.Config;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.util.AttributeSet;
import android.view.SurfaceView;

public class MagneticSurface extends SurfaceView {
	private static final int REFERENCE_MAGNETIC_FIELD = 50;

	private final int width, height;

	private final Canvas cacheCanvas;
	private final Bitmap cacheBitmap;
	
	private final Paint white;
	private final Paint green;
	private final Paint red;
	
	private String label = "??";
	
	public MagneticSurface(Context context, AttributeSet attrs) {
		super(context, attrs);
		setWillNotDraw(false);
		
		width = 100;
		height = 100;
		
		cacheBitmap = Bitmap.createBitmap(width, height, Config.ARGB_8888);
		cacheCanvas = new Canvas(cacheBitmap);

		white = new Paint();
		white.setColor(0xffffffff);
		white.setStyle(Style.STROKE);
		
		green = new Paint();
		green.setColor(0xff00ff00);
		green.setStyle(Style.STROKE);

		red = new Paint();
		red.setColor(0xffff0000);
		red.setStyle(Style.STROKE);
	}

	public void reset() {
		cacheCanvas.drawRGB(0, 0, 0);
		cacheCanvas.drawCircle(width/2, height/2, REFERENCE_MAGNETIC_FIELD, white);
		cacheCanvas.drawText(label, width/2-white.getTextSize()/2, height/2+white.getTextSize()/2, white);
		invalidate();
	}
	
	public void putPixel(int x, int y) {
		final Paint color;
		if (Math.sqrt(x * x + y * y) > REFERENCE_MAGNETIC_FIELD) {
			color = red;
		} else {
			color = green;
		}
		
		x += width/2;
		y += height/2;

		if (x < 0) {
			x = 0;
		}
		if (x > width-1) {
			x = width-1;
		}
		if (y < 0) {
			y = 0;
		}
		if (y > height-1) {
			y = height;
		}

		cacheCanvas.drawPoint(x, y, color);
		invalidate();
	}
	
	@Override
	protected void onDraw(Canvas canvas) {
		canvas.drawRGB(0, 0, 0);
		canvas.drawBitmap(cacheBitmap, 0, 0, null);
	}

	public void setLabel(String string) {
		label = string;
	}
}
