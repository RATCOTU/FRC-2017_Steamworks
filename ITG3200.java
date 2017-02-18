package org.usfirst.frc.team6236.robot;

/**
 * Copyright (c) 2015, www.techhounds.com
 * All rights reserved.
 *
 * <p>
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * </p>
 * <ul>
 * <li>Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.</li>
 * <li>Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.</li>
 * <li>Neither the name of the www.techhounds.com nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.</li>
 * </ul>
 *
 * <p>
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * </p>
 */

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A Java wrapper around the ITC based ITG-3200 triple axis gryo.
 * 
 * <p>
 * Typical usage:
 * </p>
 * <ul>
 * <li>Make sure your ITG-3200 is connected to the I2C bus on the roboRIO and
 * mounted flat (so Z axis is used to track direction robot is facing).</li>
 * <li>Construct a single instance of the {@link ITG3200} class to be shared
 * throughout your Robot code.</li>
 * <li>Use the {@link #createGyroZ()} methods to
 * track your robots rotation (direction your robot is
 * facing).</li>
 * </ul>
 * 
 * <pre><code>
 * ITG3200 sensor;
 * GyroBase gyrox;
 * GyroBase gyroy;
 * GyroBase gyroz;
 * 
 * public void robotInit() {
 * 	 sensor = new ITG3200(RobotMap.ITG3200_PORT,
 *				          RobotMap.ITG3200_JUMPERED);
 *
 *   gyrox = sensor.createGyroX();
 *   gyroy = sensor.createGyroY();
 *   gyroz = sensor.createGyroZ();
 *   
 *	 // Add Gyro objects to LiveWindow test mode
 * 	 String group = "ITG-3200";
 *   LiveWindow.addSensor(group, "Gyro (x)", gyrox);
 * 	 LiveWindow.addSensor(group, "Gyro (y)", gyroy);
 *   LiveWindow.addSensor(group, "Gyro (z)", gyroz);
 * }
 * 
 * public double getGyroAngleX() {
 *   return gyrox.getAngle();
 * }
 * 
 * public double getGyroAngleY() {
 *   return gyroy.getAngle();
 * }
 * 
 * public double getGyroAngleZ() {
 *   return gyroz.getAngle();
 * }
 * </code></pre>
 * 
 * <p>
 * Be aware of the following:
 * </p>
 * <ul>
 * <li>Angles are in signed degrees (both positive and negative values are
 * possible) and not necessarily normalized (large values like -1203 degrees are
 * possible).</li>
 * <li>The {@link #reset} method is called once initially at construction.
 * Reseting the gyro should only be done when your robot is stationary and it
 * may take up to one second. You should not need to do this and should avoid
 * doing this during the autonomous or teleop periods (unless you know your
 * robot won't be moving).</li>
 * <li>There is a background thread that is automatically started that keeps
 * reading and accumulating values from the ITG-3200. You should not need to use
 * the {@link #start()} or {@link #stop()} methods during normal matches.
 * However, if you only use the gyro during the autonomous period, you can use
 * the {@link #stop()} method at the end of the autonomous period to save some
 * CPU.</li>
 * </ul>
 * 
 * <h2>Suggested Usage</h2>
 * <p>
 * You should be able to use this class to aid your robot in making relative
 * turns. For example, if you want to create a command which rotates your robot
 * 90 degrees.
 * </p>
 * <ul>
 * <li>In your command's initialize method, use the {@link #createGyroZ()} to construct a {@link Gyro} instance where the current direction is treated as zero.</li>
 * <li>Use the {@link Gyro} object as a PID source and/or check the current
 * angle reported by the {@link Gyro#getAngle()} method in the Gyro object in your isFinished() method.</li>
 * </ul>
 */
public final class ITG3200{

	// List of I2C registers which the ITG-3200 uses from the datasheet: https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
	private static final byte WHO_AM_I = 0x00;
	private static final byte SMPLRT_DIV = 0x15;
	private static final byte DLPF_FS = 0x16;
	// private static final byte INT_CFG = 0x17;
	// private static final byte INT_STATUS = 0x1A;
	// private static final byte TEMP_OUT_H = 0x1B;
	// private static final byte TEMP_OUT_L = 0x1C;
	private static final byte GYRO_XOUT_H = 0x1D;
	// private static final byte GYRO_XOUT_L = 0x1E;
	// private static final byte GYRO_YOUT_H = 0x1F;
	// private static final byte GYRO_YOUT_L = 0x20;
	// private static final byte GYRO_ZOUT_H = 0x21;
	// private static final byte GYRO_ZOUT_L = 0x22;

	//
	// Bit flags used for interrupt operation
	//

	/*
	 * Set this bit in INT_CFG register for logic level of INT output pin to be
	 * low when interrupt is active (leave 0 if you want it high).
	 */
	// private static final byte INT_CFG_ACTL_LOW = (byte) (1 << 7);

	/*
	 * Set drive type for interrupt to open drain mode, omit if you want
	 * push-pull mode (what does this mean?).
	 */
	// private static final byte INT_CFG_OPEN_DRAIN = 1 << 6;

	/*
	 * Interrupt latch mode (remains set until you clear it), omit this flag to
	 * get a 0-50us interrupt pulse.
	 */
	// private static final byte INT_CFG_LATCH_INT_EN = 1 << 5;

	/*
	 * Allow any read operation of data to clear the interrupt flag (otherwise
	 * it is only cleared after reading status register).
	 */
	// private static final byte INT_CFG_ANYRD_2CLEAR = 1 << 4;

	/*
	 * Enable interrup when device is ready (PLL ready after changing clock
	 * source). Hmmm?
	 */
	// private static final byte INT_CFG_RDY_EN = 1 << 3;

	/* Enable interrupt when new data is available. */
	// private static final byte INT_CFG_RAW_RDY_EN = 1 << 1;

	/* Guess at mode to use if we want to try enabling interrupts. */
	// private static final byte INT_CFG_SETTING = (INT_CFG_LATCH_INT_EN |
	// INT_CFG_ANYRD_2CLEAR | INT_CFG_RAW_RDY_EN);

	//
	// The bit flags that can be set in the DLPF register on the ITG-3200
	// as specified in the ITG-3200 data sheet.
	//

	//
	// The low pass filter bandwidth settings
	//

	// private static final byte DLPF_LOWPASS_256HZ = 0 << 0;
	private static final byte DLPF_LOWPASS_188HZ = 1 << 0;
	// private static final byte DLPF_LOWPASS_98HZ = 2 << 0;
	// private static final byte DLPF_LOWPASS_42HZ = 3 << 0;
	// private static final byte DLPF_LOWPASS_20HZ = 4 << 0;
	// private static final byte DLPF_LOWPASS_10HZ = 5 << 0;
	// private static final byte DLPF_LOWPASS_5HZ = 6 << 0;

	/** Select range of +/-2000 deg/sec. (only range supported). */
	private static final byte DLPF_FS_SEL_2000 = 3 << 3;

	/**
	 * The I2C address of the ITG-3200 when AD0 (pin 9) is jumpered to logic
	 * high.
	 */
	private static final byte itgAddressJumper = 0x69;

	/**
	 * The I2C address of the ITG-3200 when AD0 (pin 9) is jumpered to logic
	 * low.
	 */
	private static final byte itgAddressNoJumper = 0x68;

	/** Multiplier to convert raw integer values returned to degrees/sec. */
	private static final float COUNT_TO_DEGSEC = (float) (1.0 / 14.375);

	/** Set this to true for lots of diagnostic output. */
	private static final boolean DEBUG = false;

	/**
	 * How many sample readings to make to determine the bias value for each
	 * axis.
	 */
	private static final int MIN_READINGS_TO_SET_BIAS = 50;

	/** I2C Address to use to communicate with the ITG-3200. */
	private byte m_addr;

	/** I2C object used to communicate with Gyro. */
	private I2C m_i2c;

	/**
	 * Background thread responsible for accumulating angle data from the
	 * sensor.
	 */
	private Thread m_thread;

	/** Flag used to signal background thread that the gyro should be reset. */
	private boolean m_need_reset;

	/** Accumulator for rotation around the x-axis. */
	private Accumulator m_x;

	/** Accumulator for rotation around the y-axis. */
	private Accumulator m_y;

	/** Accumulator for rotation around the z-axis. */
	private Accumulator m_z;
	private int[] m_xBuffer;
	private int[] m_yBuffer;
	private int[] m_zBuffer;
	private int m_cntBuffer;
	private int m_sizeBuffer;
	private double[] m_timeBuffer;

	/**
	 * Construct a new instance of the ITG-3200 gryo class.
	 * 
	 * <p>
	 * IMPORTANT
	 * 
	 * @param port
	 *            This should be {@link I2C.Port#kOnboard} if the ITG-3200 is
	 *            connected to the main I2C bus on the roboRIO. This should be
	 *            {@link I2C.Port#kMXP} if it is connected to the I2C bus on the
	 *            MXP port on the roboRIO.
	 * @param jumper
	 *            This should be true if the ITG-3200 has the AD0 jumpered to
	 *            logic level high and false if not.
	 */
	public ITG3200(I2C.Port port, boolean jumper) {
		m_addr = (jumper ? itgAddressJumper : itgAddressNoJumper);
		m_i2c = new I2C(port, m_addr);
		if (DEBUG) {
			check();
		}
		m_x = new Accumulator();
		m_y = new Accumulator();
		m_z = new Accumulator();
		m_need_reset = true;

		start();
	}

	/**
	 * Returns a pseudo {@class Gyro} instance that you can use to monitor
	 * rotation about the x-axis.
	 * 
	 * <p>
	 * NOTE: The {@class Gyro} returned has some limitations/features:
	 * </p>
	 * 
	 * <ul>
	 * <li>The {@link Gyro#calibrate()} method does nothing.</li>
	 * <li>The {@link Gyro#reset()} method only resets that instance to zero
	 * (any other Gyro objects created will be unchanged). This is typically
	 * what you want.</li>
	 * <li>The {@link Gyro#getRate()} method is implemented (you can use it).</li>
	 * </ul>
	 * 
	 * @return A new {@class Gyro} object you can used for tracking rotation.
	 */
	public GyroAdapter createGyroX() {
		return new GyroAdapter(m_x.getDegrees(), true) {
			@Override
			protected double getSensorValue() {
				return m_x.getDegrees();
			}

			@Override
			public double getRate() {
				return m_x.getDegPerSec();
			}
		};
	}

	/**
	 * Returns a pseudo {@class Gyro} instance that you can use to monitor
	 * rotation about the y-axis.
	 * 
	 * <p>
	 * NOTE: The {@class Gyro} returned has some limitations/features:
	 * </p>
	 * 
	 * <ul>
	 * <li>The {@link Gyro#calibrate()} method does nothing.</li>
	 * <li>The {@link Gyro#reset()} method only resets that instance to zero
	 * (any other Gyro objects created will be unchanged). This is typically
	 * what you want.</li>
	 * <li>The {@link Gyro#getRate()} method is implemented (you can use it).</li>
	 * </ul>
	 * 
	 * @return A new {@class Gyro} object you can used for tracking rotation.
	 */
	public GyroAdapter createGyroY() {
		return new GyroAdapter(m_y.getDegrees(), true) {
			@Override
			protected double getSensorValue() {
				return m_y.getDegrees();
			}

			@Override
			public double getRate() {
				return m_y.getDegPerSec();
			}
		};
	}

	/**
	 * Returns a pseudo {@class Gyro} instance that you can use to monitor
	 * rotation about the z-axis.
	 * 
	 * <p>
	 * NOTE: The {@class Gyro} returned has some limitations/features:
	 * </p>
	 * 
	 * <ul>
	 * <li>The {@link Gyro#calibrate()} method does nothing.</li>
	 * <li>The {@link Gyro#reset()} method only resets that instance to zero
	 * (any other Gyro objects created will be unchanged). This is typically
	 * what you want.</li>
	 * <li>The {@link Gyro#getRate()} method is implemented (you can use it).</li>
	 * </ul>
	 * 
	 * @return A new {@class Gyro} object you can used for tracking rotation.
	 */
	public GyroAdapter createGyroZ() {
		return new GyroAdapter(m_z.getDegrees(), true) {
			@Override
			protected double getSensorValue() {
				return m_z.getDegrees();
			}

			@Override
			public double getRate() {
				return m_z.getDegPerSec();
			}
		};
	}

	/**
	 * Returns string representation of the object for debug purposes.
	 */
	public String toString() {
		return "Gyro[0x" + Integer.toHexString(m_addr & 0xff) + "]";
	}

	/**
	 * Dumps information about the state of the Gyro to the smart dashboard.
	 * 
	 * @param tag
	 *            Short name like "Gyro" to prefix each label with on the
	 *            dashboard.
	 * @param debug
	 *            Pass true if you want a whole lot of details dumped onto the
	 *            dashboard, pass false if you just want the direction of each
	 *            axis and the temperature.
	 */
	public void updateDashboard(String tag, boolean debug) {
		SmartDashboard.putNumber(tag + " x-axis degrees", m_x.getDegrees());
		SmartDashboard.putNumber(tag + " y-axis degrees", m_y.getDegrees());
		SmartDashboard.putNumber(tag + " z-axis degrees", m_z.getDegrees());

		if (debug) {
			SmartDashboard.putNumber(tag + " x-axis raw", m_x.getRaw());
			SmartDashboard.putNumber(tag + " y-axis raw", m_y.getRaw());
			SmartDashboard.putNumber(tag + " z-axis raw", m_z.getRaw());

			SmartDashboard.putNumber(tag + " x-axis count", m_x.getReadings());
			SmartDashboard.putNumber(tag + " y-axis count", m_y.getReadings());
			SmartDashboard.putNumber(tag + " z-axis count", m_z.getReadings());

			SmartDashboard.putString(tag + " I2C Address",
					"0x" + Integer.toHexString(m_addr));
		}
	}

	/**
	 * Clears dashboard variables from network table.
	 * 
	 * @param tag
	 *            Short name like "Gyro" to prefix each label with on the
	 *            dashboard (what you passed to {@link #updateDashboard}.
	 */
	public void clearDashboard(String tag) {
		NetworkTable sd = NetworkTable.getTable("SmartDashboard");
		
		sd.delete(tag + " x-axis degrees");
		sd.delete(tag + " y-axis degrees");
		sd.delete(tag + " z-axis degrees");

		sd.delete(tag + " x-axis raw");
		sd.delete(tag + " y-axis raw");
		sd.delete(tag + " z-axis raw");

		sd.delete(tag + " x-axis count");
		sd.delete(tag + " y-axis count");
		sd.delete(tag + " z-axis count");

		sd.delete(tag + " I2C Address");
	}
	
	public double getX(){
		return m_x.getDegrees();
	}
	
	public double getY(){
		return m_y.getDegrees();
	}
	
	public double getZ(){
		return m_z.getDegrees();
	}
	
	
	/**
	 * Internal method that runs in the background thread to accumulate data
	 * from the Gyro.
	 */
	private void accumulateData() {
		int resetCnt = 0;
		Thread thisThread = Thread.currentThread();

		while (thisThread.isInterrupted() == false) {
			if (m_need_reset) {
				// Set gyro to the proper mode
				performResetSequence();

				// Reset accumulators and set the number of readings to take to
				// compute bias values
				resetCnt = MIN_READINGS_TO_SET_BIAS;
				m_x.reset();
				m_y.reset();
				m_z.reset();
				m_need_reset = false;
			} else {
				// Go read raw values from ITG-3200 and update our accumulators
				readRawAngleBytes();

				if (resetCnt > 0) {
					// If we were recently reset, and have made enough initial
					// readings,
					// then go compute and set our new bias (correction) values
					// for each accumulator
					resetCnt--;
					if (resetCnt == 0) {
						m_x.setBiasByAccumulatedValues();
						m_y.setBiasByAccumulatedValues();
						m_z.setBiasByAccumulatedValues();
					}
				}
			}

			// Short delay between each reading
			Timer.delay(.01);
		}
	}

	/**
	 * Singles the gyro's background thread that we want to reset the gyro.
	 * 
	 * <p>
	 * You don't typically need to call this during a match. If you do call it,
	 * you should only do so when the robot is stationary and will remain
	 * stationary for a short time.
	 * </p>
	 */
	public void reset() {
		m_need_reset = true;
	}

	/**
	 * Starts up the background thread that accumulates gyro statistics.
	 * 
	 * <p>
	 * You never need to call this method unless you have stopped the gyro and
	 * now want to start it up again. If you do call this method, you should
	 * probably also call the {@link #reset} method.
	 * </p>
	 */
	public void start() {

		if (m_thread != null) {
			stop();
		}

		m_thread = new Thread(new Runnable() {
			@Override
			public void run() {
				accumulateData();
			}
		});
		m_thread.start();
	}

	/**
	 * Stops the background thread from accumulating angle information (turns
	 * OFF gyro!).
	 * 
	 * <p>
	 * This method is not typically called as it stops the gyro from
	 * accumulating statistics essentially turning it off. The only time you
	 * might want to do this is if you are done using the gyro for the rest of
	 * the match and want to save some CPU cyles (for example, if you only
	 * needed the gyro during the autonomous period).
	 * </p>
	 */
	public void stop() {
		if (m_thread != null) {
			try {
				m_thread.interrupt();
			} catch (Error ignore) { }
			m_thread = null;
		}
	}

	/**
	 * Sends commands to configure the ITG-3200 the way we need it to run.
	 */
	private void performResetSequence() {
		// Configure the gyroscope
		// Set the gyroscope scale for the outputs to +/-2000 degrees per second
		m_i2c.write(DLPF_FS, (DLPF_FS_SEL_2000 | DLPF_LOWPASS_188HZ));
		// Set the sample rate to 100 hz
		m_i2c.write(SMPLRT_DIV, 9);
	}

	/**
	 * Enables the buffering of the next "n" data samples (which can then be
	 * saved for analysis).
	 * 
	 * @param samples
	 *            Maximum number of samples to read.
	 */
	public void enableBuffer(int samples) {
		double[] timeBuffer = new double[samples];
		int[] xBuffer = new int[samples];
		int[] yBuffer = new int[samples];
		int[] zBuffer = new int[samples];
		synchronized (this) {
			m_timeBuffer = timeBuffer;
			m_xBuffer = xBuffer;
			m_yBuffer = yBuffer;
			m_zBuffer = zBuffer;
			m_cntBuffer = 0;
			m_sizeBuffer = samples;
		}
	}

	/**
	 * Check to see if the buffer is full.
	 * 
	 * @return true if buffer is at capacity.
	 */
	public boolean isBufferFull() {
		boolean isFull;
		synchronized (this) {
			isFull = (m_cntBuffer == m_sizeBuffer);
		}
		return isFull;
	}

	/**
	 * Writes any raw buffered data to the file "/tmp/gyro-data.csv" for
	 * inspection via Excel.
	 */
	public void saveBuffer() {
		double[] timeBuffer;
		int[] xBuffer;
		int[] yBuffer;
		int[] zBuffer;
		int size;

		// Transfer buffer info to local variables and turn off buffering in a
		// thread safe way.
		synchronized (this) {
			timeBuffer = m_timeBuffer;
			xBuffer = m_xBuffer;
			yBuffer = m_yBuffer;
			zBuffer = m_zBuffer;
			size = m_cntBuffer;
			m_sizeBuffer = 0;
			m_cntBuffer = 0;
		}

		if (size > 0) {
			try {
				PrintStream out = new PrintStream(
						new File("/tmp/gryo-data.csv"));
				out.println("\"FPGA Time\",\"x-axis\",\"y-axis\",\"z-axis\"");
				for (int i = 0; i < size; i++) {
					out.println(timeBuffer[i] + "," + xBuffer[i] + ","
							+ yBuffer[i] + "," + zBuffer[i]);
				}
				out.close();
				SmartDashboard.putBoolean("Gyro Save OK", true);
			} catch (IOException ignore) {
				SmartDashboard.putBoolean("Gyro Save OK", false);
			}
		}
	}

	/**
	 * Internal method run in the background thread that reads values from the
	 * ITG-3200 and updates the accumulators.
	 */
	private void readRawAngleBytes() {
		double now = Timer.getFPGATimestamp();

		byte[] buffer = new byte[6];
		boolean rc = m_i2c.read(GYRO_XOUT_H, buffer.length, buffer);

		if (rc) {
			// Got a good read, get 16 bit integer values for each axis and
			// update accumulated values
			int x = (buffer[0] << 8) | (buffer[1] & 0xff);
			int y = (buffer[2] << 8) | (buffer[3] & 0xff);
			int z = (buffer[4] << 8) | (buffer[5] & 0xff);

			m_x.update(x, now);
			m_y.update(y, now);
			m_z.update(z, now);

			// If buffered enabled, then save values in a thread safe way
			if (m_sizeBuffer > 0) {
				synchronized (this) {
					int i = m_cntBuffer;
					if (i < m_sizeBuffer) {
						m_timeBuffer[i] = now;
						m_xBuffer[i] = x;
						m_yBuffer[i] = y;
						m_zBuffer[i] = z;
						m_cntBuffer++;
					}
				}
			}
		}

		if (DEBUG) {
			String name = toString();
			String[] labels = { "XOUT_H", "XOUT_L", "YOUT_H", "YOUT_L",
					"ZOUT_H", "ZOUT_L" };
			for (int i = 0; i < labels.length; i++) {
				SmartDashboard.putString(name + " " + labels[i],
						"0x" + Integer.toHexString(0xff & buffer[i]));
			}
		}
	}

	/**
	 * Helper method to check that we can communicate with the gyro.
	 */
	private void check() {
		byte[] buffer = new byte[1];
		boolean rc = m_i2c.read(WHO_AM_I, buffer.length, buffer);
		if (DEBUG) {
			String name = toString();
			SmartDashboard.putBoolean(name + " Check OK?", rc);
			SmartDashboard.putNumber(name + " WHO_AM_I", buffer[0]);
		}
	}

	/**
	 * Private helper class to accumulate values read from the gryo and convert
	 * degs/sec into degrees.
	 */
	private class Accumulator {
		/** Accumulated degrees since zero. */
		private double m_accumulatedDegs;
		/**
		 * 2 times the computed bias value that is used when getting average of
		 * readings.
		 */
		private double m_bias2;
		/** The prior raw value read from the gyro. */
		private int m_lastRaw;
		/** The prior time stamp the last raw value was read. */
		private double m_lastTime;
		/** The total count of time the gyro value has been read. */
		private int m_cnt;
		/** The sum of all of the raw values read. */
		private long m_sum;

		/** Multipler to covert 2*Count to degrees/sec (optimization). */
		private static final double COUNT2_TO_DEGSEC = (COUNT_TO_DEGSEC / 2.0);

		/**
		 * Returns the accumulated degrees.
		 * 
		 * @return Accumulated signed degrees since last zeroed.
		 */
		public synchronized double getDegrees() {
			return m_accumulatedDegs;
		}

		/**
		 * @return The raw integer reading from the ITG-3200 associated with the
		 *         axis.
		 */
		public int getRaw() {
			return m_lastRaw;
		}

		/**
		 * Returns the deg/sec value based on the last reading from the sensor.
		 * 
		 * @return Degrees/Second reported by the sensor.
		 */
		public double getDegPerSec() {
			return (m_lastRaw - (m_bias2 / 2.0)) * COUNT_TO_DEGSEC;
		}

		/**
		 * Returns the number or readings that went into the accumulated
		 * degrees.
		 * 
		 * @return Count of readings since last zeroed.
		 */
		public synchronized int getReadings() {
			return m_cnt;
		}

		/**
		 * Constructs a new instance.
		 */
		private Accumulator() {
			m_bias2 = 0;
			zero();
		}

		/**
		 * Zeros out accumulated information.
		 */
		private void zero() {
			m_lastRaw = 0;
			m_lastTime = 0;
			m_sum = 0;
			synchronized (this) {
				m_cnt = 0;
				m_accumulatedDegs = 0;
			}
		}

		/**
		 * Zeros out accumulated information and clears (zeros) the internal
		 * bias value.
		 */
		private void reset() {
			zero();
			m_bias2 = 0;
		}

		/**
		 * Computes new bias value from accumulated values and then zeros.
		 */
		private void setBiasByAccumulatedValues() {
			m_bias2 = 2.0 * ((double) m_sum) / ((double) m_cnt);
			zero();
		}

		/**
		 * Updates (accumulates) new value read from axis.
		 * 
		 * @param raw
		 *            Raw signed 16 bit value read from gyro for axis.
		 * @param time
		 *            The time stamp when the value was read.
		 */
		private void update(int raw, double time) {
			double degs = 0;

			if (m_cnt != 0) {
				// Get average of degrees per second over the time span
				double degPerSec = (m_lastRaw + raw - m_bias2)
						* COUNT2_TO_DEGSEC;
				// Get time span this rate occurred for
				double secs = (m_lastTime - time);
				// Get number of degrees rotated for time period
				degs = degPerSec * secs;
			}

			// Update our thread shared values
			synchronized (this) {
				m_accumulatedDegs += degs;
				m_sum += raw;
				m_cnt++;
				m_lastRaw = raw;
				m_lastTime = time;
			}
		}
	}
}