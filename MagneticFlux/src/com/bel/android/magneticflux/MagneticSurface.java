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
	private static final int REFERENCE_MAGNETIC_FIELD = 50;

	private final int width, height;

	private final Canvas cacheCanvas;
	private final Bitmap cacheBitmap;
	
	private final Paint white, gray, green, red;
	
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
		cacheCanvas.drawCircle(width/2, height/2, REFERENCE_MAGNETIC_FIELD, white);
		
		textHeight = height/2 + white.getTextSize() / 2;
		cacheCanvas.drawText(label         , width/2          , textHeight           , white);
		textHeight = height/2 + gray.getTextSize() / 2;
		cacheCanvas.drawText("-"+(width/4) , width/2 - width/4, textHeight           , gray);
		cacheCanvas.drawText("+"+(width/4) , width/2 + width/4, textHeight           , gray);
		cacheCanvas.drawText("-"+(height/4), width/2          , textHeight - height/4, gray);
		cacheCanvas.drawText("+"+(height/4), width/2          , textHeight + height/4, gray);
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
