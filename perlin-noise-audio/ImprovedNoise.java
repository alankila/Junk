import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.security.SecureRandom;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.LineUnavailableException;

public final class ImprovedNoise {
	public static void main(String[] args) throws LineUnavailableException, InterruptedException, IOException {
		int octaves = 8;
		int pos = 0;
		int clipLen = 48000;
		double[] audio = new double[clipLen * octaves];
		for (int k = 1; k <= octaves; k ++) {
			/* Produce 100 Hz base tone. Be careful to not let
			 * frequencyDivider grow up to 1.0 at any octave to avoid aliasing the noise. */
			double frequencyDivider = 100.0 / clipLen;
			
			/* Compute normalization factor assuming the output from noise() is -1 to 1 maximally. */
			double valueDivider = 1;
			double sum = 0;
			for (int j = 0; j < k; j ++) {
				sum += valueDivider;
				valueDivider /= 2;
			}
			valueDivider = 1 / sum;

			for (int j = 0; j < k; j ++) {
				for (int i = 0; i < clipLen; i ++) {
					audio[pos + i] += noise((pos + i) * frequencyDivider) * valueDivider;
				}
				frequencyDivider *= 2;
				valueDivider /= 2;
			}
			pos += clipLen;
		}
		
		byte[] wavData = new byte[audio.length * 2];
		for (int i = 0; i < audio.length; i ++) {
			int value = (int) (audio[i] * 32767.0);
			wavData[i * 2 + 0] = (byte) (value >> 8);
			wavData[i * 2 + 1] = (byte) value;
		}

		Files.write(Paths.get("audio.raw"), wavData);
		
		Clip clip = AudioSystem.getClip();
		clip.open(new AudioFormat(clipLen, 16, 1, true, true), wavData, 0, wavData.length);
		clip.start();
		/* This short sleep makes sure audio clip actually has time to start. Otherwise drain may just skip whole audio. */
		Thread.sleep(100);
		clip.drain();
		clip.close();
	}
	
	private static double noise(double x) {
		int X = (int) Math.floor(x) & (SIZE - 1);
		x -= Math.floor(x);
		double u = fade(x);
	    int A = p[X];
	    int AA = p[A];
	    int B = p[X+1];
	    int BA = p[B];
	    return lerp(u, grad(p[AA], x), grad(p[BA], x - 1));
	}

	private static double fade(double t) {
		return t * t * t * (t * (t * 6 - 15) + 10);
	}

	private static double lerp(double t, double a, double b) {
		return a + t * (b - a);
	}
	
	private static double grad(int hash, double x) {
		/* this is the traditional gradient noise function, where gradients always use maximum amplitude. */
		return (hash & 1) != 0 ? x : -x;
		/* hash value range is from 1 to SIZE-1, so its average is SIZE/2 and deviates up to SIZE-1 from it */
		//return (hash - (SIZE / 2)) * x / (SIZE / 2 - 1);
	}

	private static final int SIZE = 65536;
	private static final int p[] = new int[SIZE * 2];
	static {
		SecureRandom sr = new SecureRandom();
		for (int i = 0; i < SIZE ; i++) {
			p[SIZE + i] = p[i] = 1 + sr.nextInt(SIZE - 1);
		}
	}
}
