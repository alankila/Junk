package com.bel.android.magneticflux;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Bitmap.Config;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Paint.Align;
import android.graphics.Paint.Style;
import android.util.AttributeSet;
import android.view.SurfaceView;

public class MagneticSurface extends SurfaceView {
	private final int radius;
	private int range;

	private final Canvas cacheCanvas;
	private final Bitmap cacheBitmap;
	
	private final Paint white, gray, green, red;
	
	private String label = "??";
	
	public MagneticSurface(Context context, AttributeSet attrs) {
		super(context, attrs);
		setWillNotDraw(false);
		
		radius = 100;
		
		cacheBitmap = Bitmap.createBitmap(radius, radius, Config.ARGB_8888);
		cacheCanvas = new Canvas(cacheBitmap);

		white = new Paint();
		white.setColor(0xffffffff);
		white.setStyle(Style.STROKE);
		white.setTextAlign(Align.CENTER);
		
		gray = new Paint();
		gray.setColor(0x88ffffff);
		gray.setStyle(Style.STROKE);
		gray.setTextSize(9);
		gray.setTextAlign(Align.CENTER);

		green = new Paint();
		green.setColor(0x4400ff00);
		green.setStyle(Style.STROKE);

		red = new Paint();
		red.setColor(0x44ff0000);
		red.setStyle(Style.STROKE);
	}

	public void reset() {
		float textHeight;
		
		cacheCanvas.drawRGB(0, 0, 0);
		cacheCanvas.drawCircle(radius/2, radius/2, radius/2, white);
		
		textHeight = radius/2 + white.getTextSize() / 2;
		cacheCanvas.drawText(label        , radius/2          , textHeight          , white);
		textHeight = radius/2 + gray.getTextSize() / 2;
		cacheCanvas.drawText("-"+(range/2), radius/2 - radius/4, textHeight          , gray);
		cacheCanvas.drawText("+"+(range/2), radius/2 + radius/4, textHeight          , gray);
		cacheCanvas.drawText("-"+(range/2), radius/2          , textHeight - radius/4, gray);
		cacheCanvas.drawText("+"+(range/2), radius/2          , textHeight + radius/4, gray);
		
		invalidate();
	}
	
	public void putPixel(float x, float y) {
		final Paint color;
		if (Math.sqrt(x * x + y * y) > range) {
			color = red;
		} else {
			color = green;
		}
		
		x /= range * 2;
		y /= range * 2;

		x += 0.5f;
		y += 0.5f;
		
		x *= radius;
		y *= radius;

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

	public void setRange(int i) {
		range = i;
	}
}
