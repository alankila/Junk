import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import javax.imageio.ImageIO;

public class Mandelbrot {
	protected static final int ORBITS = (int) 1e7;
	protected static final int DIM = 1024;

	protected static void renderImage(float[] ra, float[] ga, float[] ba, int riter, int giter, int biter) {
		int iterations = Math.max(Math.max(riter, giter), biter);
		double[] orbit = new double[iterations * 2];
		for (int i = 0; i < ORBITS; i ++) {
			double Cre = (Math.random() - 0.5) * 4;
			double Cim = (Math.random() - 0.5) * 4;

			/* Reject two largest black areas right off the bat */
			double p = Math.sqrt(Math.pow(Cre - 1/4.0, 2) + Cim * Cim);
			if (Cre < p - 2 * p * p + 1 / 4.0 || Math.pow(Cre + 1, 2) + Cim * Cim < 1 / 16.0) {
			    continue;
			}

			double a = 1;//Math.random();
			double Zre = Cre * a;
			double Zim = Cim * a;
			
			for (int j = 0; j < iterations; j ++) {
				orbit[j * 2 + 0] = Zre;
				orbit[j * 2 + 1] = Zim;
				
				/* Escaped */
				if (Zre * Zre + Zim * Zim > 4.0) {
					for (int k = 0; k <= j; k ++) {
						int y = (int) Math.round(orbit[k * 2 + 0] * DIM / 4) + DIM / 2;
						int x = (int) Math.round(orbit[k * 2 + 1] * DIM / 4) + DIM / 2;
						if (x < 0 || x >= DIM) {
							continue;
						}
						if (y < 0 || y >= DIM) {
							continue;
						}
						if (j < riter) {
							ra[y * DIM + x] += k;
						}
						if (j < giter) {
							ga[y * DIM + x] += k;
						}
						if (j < biter) {
							ba[y * DIM + x] += k;
						}
					}
					break;
				}
				
			    double tmp = Zre * Zre - Zim * Zim + Cre;
			    Zim = 2 * Zre * Zim + Cim;
			    Zre = tmp;
			}
			
			if ((i % 1000000) == 0) {
				System.out.println("iterations: " + i);
			}
		}
	}
	
	public static void normalize(float[] picture) {
		float max = 0;
		for (int i = 0; i < picture.length; i ++) {
			max = Math.max(max, picture[i]);
		}
		for (int i = 0; i < picture.length; i ++) {
			picture[i] /= max;
		}
	}
	
	public static void main(String[] args) throws IOException {
		float[] ra = new float[DIM * DIM];
		float[] ga = new float[DIM * DIM];
		float[] ba = new float[DIM * DIM];
		renderImage(ra, ga, ba, 5000, 500, 50);
		normalize(ra);
		normalize(ga);
		normalize(ba);
		
		BufferedImage bi = new BufferedImage(DIM, DIM, BufferedImage.TYPE_3BYTE_BGR);
		for (int y = 0; y < DIM; y ++) {
			for (int x = 0; x < DIM; x ++) {
				double rr = Math.sqrt(ra[y * DIM + x]);
				double gg = Math.sqrt(ga[y * DIM + x]);
				double bb = Math.sqrt(ba[y * DIM + x]);
				int r = (int) Math.round(rr * 255.0);
				int g = (int) Math.round(gg * 255.0);
				int b = (int) Math.round(bb * 255.0);
				r = Math.min(r, 255);
				g = Math.min(g, 255);
				b = Math.min(b, 255);
				bi.setRGB(x, y, 0xff000000 | r << 16 | g << 8 | b);
			}
		}
		try (FileOutputStream fos = new FileOutputStream("test2.png")) {
			ImageIO.write(bi, "PNG", fos);
		}
		new File("test2.png").renameTo(new File("test.png"));
		
		System.out.println("done");
	}
}
