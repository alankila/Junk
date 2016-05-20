package fi.bel.rj.util;

import java.awt.image.BufferedImage;
import java.io.FileOutputStream;
import java.io.IOException;

import javax.imageio.ImageIO;

public class Mandelbrot {
	protected static final int ORBITS = (int) 1e7;
	protected static final int ITER = 10240;
	protected static final int DIM = 1024;

	public static void main(String[] args) throws IOException {
		float[] picture = new float[DIM * DIM];
		
		float[] orbit = new float[ITER * 2];
		for (int i = 0; i < ORBITS; i ++) {
			float Cre = (float) (Math.random() - 0.5) * 4;
			float Cim = (float) (Math.random() - 0.5) * 4;
			float a = 1;//(float) Math.random();
			float Zre = Cre * a;
			float Zim = Cim * a;
			
			for (int j = 0; j < ITER; j ++) {
				orbit[j * 2 + 0] = Zre;
				orbit[j * 2 + 1] = Zim;
				
				/* Escaped */
				if (Zre * Zre + Zim * Zim > 4) {
					//System.out.println("orbit at " + j);
					for (int k = 0; k <= j; k ++) {
						int y = Math.round(orbit[k * 2 + 0] * DIM / 4) + DIM / 2;
						int x = Math.round(orbit[k * 2 + 1] * DIM / 4) + DIM / 2;
						if (x < 0 || x >= DIM) {
							continue;
						}
						if (y < 0 || y >= DIM) {
							continue;
						}
						picture[y * DIM + x] += k;
					}
					break;
				}
				
			    float tmp = Zre * Zre - Zim * Zim + Cre;
			    Zim = 2 * Zre * Zim + Cim;
			    Zre = tmp;
			}
			
			if ((i % 10000000) == 0) {
				System.out.println("iterations: " + i);
			}
		}
		
		float max = 0;
		for (int i = 0; i < picture.length; i ++) {
			max = Math.max(max, picture[i]);
		}
			
		BufferedImage bi = new BufferedImage(DIM, DIM, BufferedImage.TYPE_3BYTE_BGR);
		for (int y = 0; y < DIM; y ++) {
			for (int x = 0; x < DIM; x ++) {
				float color = picture[y * DIM + x] / max * 65536;
				int r = (int) Math.sqrt(color);
				if (r > 255) {
					r = 255;
				}
				int g = (int) Math.sqrt(color * 5);
				if (g > 255) {
					g = 255;
				}
				int b = (int) Math.sqrt(color * 10);
				if (b > 255) {
					b = 255;
				}
				bi.setRGB(x, y, 0xff000000 | r << 16 | g << 8 | b);
			}
		}
		try (FileOutputStream fos = new FileOutputStream("/Users/alankila/test.png")) {
			ImageIO.write(bi, "PNG", fos);
		}
		
		System.out.println("done");
	}
}
