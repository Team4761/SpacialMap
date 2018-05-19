package frc.team166.chopshoplib;

import java.nio.ByteBuffer;
import java.util.Optional;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Lidar extends SensorBase implements PIDSource {
    I2C i2cDevice;
    Thread t;
    private double distanceMM;

    private boolean isValid;
    private double samples[];
    private int sampleIndex;
    private boolean reset;

    private StandardDeviation sd;
    private double standardDeviationValue;
    double standardDeviationLimit;

    private class PollSensor implements Runnable {
        public void run() {
            while (true) {
                /* Get the distance from the sensor */
                readDistance();
                /* Sensor updates at 60Hz, but we'll run this at 50 since the math is nicer */
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    /* We stop for nothing! */
                }
            }
        }
    }

    public static class Settings {
        public enum opMode {
            SINGLESTEP, CONTINOUS, INVALID
        }

        public enum ledIndicator {
            ON, OFF, MEASUREMENT
        }

        public enum presetConfiguration {
            HIGHSPEED, LONGRANGE, HIGHACCURACY, TINYLIDAR, CUSTOM
        }

        public enum offsetCalFlag {
            CUSTOM, DEFAULT
        }

        public opMode operationMode;
        public presetConfiguration preset;
        public double signalRateLimit;
        public int sigmaEstimateLimate;
        public int timingBudgetInMS;
        public int preRangeVcselPeriod;
        public int finalRangeVcselPeriod;
        public String fwVersion;
        public String stPalApi;
        public offsetCalFlag offsetCalibration;
        public ledIndicator ledIndicatorMode;
        public boolean watchdogTimer;
        public int offsetCalibrationValue;
        public int crosstalkCalibrationValue;

        /**
         * This will process the response from a settings query.
         * 
         * This will process the byte array and turn it into a more easily accessible object. 
         * 
         * @param response A byte array with the response from a settings query
         */
        Settings(byte[] response) {
            /* Process the zeroth byte */
            if (response[0] == 0x43) {
                operationMode = opMode.CONTINOUS;
            } else if (response[0] == 0x53) {
                operationMode = opMode.SINGLESTEP;
            } else {
                operationMode = opMode.INVALID;
            }
            /* Process the first byte */
            switch (response[1]) {
            case 0x53:
                preset = presetConfiguration.HIGHSPEED;
                break;
            case 0x52:
                preset = presetConfiguration.LONGRANGE;
                break;
            case 0x41:
                preset = presetConfiguration.HIGHACCURACY;
                break;
            case 0x54:
                preset = presetConfiguration.TINYLIDAR;
                break;
            default:
                preset = presetConfiguration.CUSTOM;
            }
            /* Process the 2nd & 3rd bytes */
            signalRateLimit = ByteBuffer.wrap(response, 2, 2).getShort() / 65536.0;
            /* Process the 4th byte */
            sigmaEstimateLimate = response[4];
            /* Process the 5th & 6th bytes */
            timingBudgetInMS = ByteBuffer.wrap(response, 5, 2).getShort();
            /* Process the 7th byte */
            if (response[7] == 0x0e) {
                preRangeVcselPeriod = 14;
                finalRangeVcselPeriod = 10;
            } else if (response[7] == 0x12) {
                preRangeVcselPeriod = 18;
                finalRangeVcselPeriod = 14;
            }
            /* Process the 8th, 9th & 10th bytes */
            fwVersion = String.format("%d.%d.%d", response[8], response[9], response[10]);
            /* Process the 11th, 12th, & 13th bytes */
            stPalApi = String.format("%d.%d.%d", response[11], response[12], response[13]);
            /* Process the 14th byte */
            if (((response[14] >> 3) & 1) != 0) {
                offsetCalibration = offsetCalFlag.CUSTOM;
            } else {
                offsetCalibration = offsetCalFlag.DEFAULT;
            }
            switch ((response[14] & 0x6) >> 1) {
            case 0:
                ledIndicatorMode = ledIndicator.OFF;
                break;
            case 1:
                ledIndicatorMode = ledIndicator.ON;
                break;
            case 2:
                ledIndicatorMode = ledIndicator.MEASUREMENT;
                break;
            }
            if ((response[14] & 1) != 0) {
                watchdogTimer = true;
            } else {
                watchdogTimer = false;
            }
            /* Process the 15th, 16th, 17th, & 18th bytes */
            offsetCalibrationValue = ByteBuffer.wrap(response, 15, 4).getInt() / 1000;
            /* Process the 19th, 20th, 21th, & 22th bytes */
            crosstalkCalibrationValue = ByteBuffer.wrap(response, 19, 4).getInt() / 65536;
        }

    }

    /**
     * Create a LIDAR object
     * 
     * @param port The I2C port the sensor is connected to
     * @param kAddress The I2C address the sensor is found at
     * @param averageOver The number of samples to average
     */
    public Lidar(Port port, int kAddress, int averageOver) {
        i2cDevice = new I2C(port, kAddress);
        setName("Lidar", kAddress);

        // Objects related to statistics
        samples = new double[averageOver];
        sd = new StandardDeviation();
        standardDeviationLimit = 100;
        reset = false;

        t = new Thread(new PollSensor());
        t.setName(String.format("LiDAR-0x%x", kAddress));
        t.start();
    }

    /**
     * Create a LIDAR object
     * 
     * @param port The I2C port the sensor is connected to
     * @param kAddress The I2C address the sensor is found at
     */
    public Lidar(Port port, int kAddress) {
        // Default to averaging over 10 samples
        this(port, kAddress, 25);
    }

    /**
     * Set the maximum allowed standard deviation before the input is considered invalid
     * 
     * @param sdLimit The maximum standard deviation expected
     */
    public void setStandardDeviationLimit(double sdLimit) {
        standardDeviationLimit = sdLimit;
    }

    /**
     * Clear the samples
     */
    public synchronized void reset() {
        for (int i = 0; i < samples.length; i++) {
            samples[i] = 0;
        }
        sampleIndex = 0;
        reset = true;
    }

    /**
     * This function gets the distance from a LiDAR sensor
     * @param bFlag True requests the distance in inches, false requests the distance in mm
     */
    public Optional<Double> getDistanceOptional(Boolean bFlag) {
        if (isValid == false) {
            return Optional.empty();
        }
        if (bFlag == true) {
            return Optional.of((distanceMM / 25.4));
        } else {
            return Optional.of(new Double(distanceMM));
        }
    }

    /**
     * This function gets the distance from a LiDAR sensor
     * @param bFlag True requests the distance in inches, false requests the distance in mm
     */
    public double getDistance(Boolean bFlag) {
        if (bFlag == true) {
            return distanceMM / 25.4;
        } else {
            return distanceMM;
        }
    }

    private void readDistance() {
        byte[] dataBuffer = new byte[2];

        i2cDevice.write(0x44, 0x1);
        i2cDevice.readOnly(dataBuffer, 2);
        ByteBuffer bbConvert = ByteBuffer.wrap(dataBuffer);
        synchronized (this) {
            samples[sampleIndex] = bbConvert.getShort();
            sampleIndex++;
            if (sampleIndex == samples.length) {
                reset = false;
                sampleIndex = 0;
            }
            distanceMM = new Mean().evaluate(samples, 0, reset ? sampleIndex : samples.length);
            // If the standard deviation is really high then the sensor likely doesn't have a valid reading.
            standardDeviationValue = sd.evaluate(samples, 0, reset ? sampleIndex : samples.length);
            if (standardDeviationValue >= standardDeviationLimit) {
                isValid = false;
            } else {
                isValid = true;
            }
        }
    }

    /**
     * Change the mode of the LiDAR sensor
     * @param mode Which mode to change to
     */
    public void setMode(Settings.opMode mode) {
        if (mode == Settings.opMode.CONTINOUS) {
            i2cDevice.writeBulk(new byte[] { 0x4d, 0x43 });
        } else if (mode == Settings.opMode.SINGLESTEP) {
            i2cDevice.writeBulk(new byte[] { 0x4d, 0x53 });
        }
    }

    public Settings querySettings() {
        byte[] dataBuffer = new byte[23];
        i2cDevice.writeBulk(new byte[] { 0x51 });
        i2cDevice.readOnly(dataBuffer, 23);
        return new Settings(dataBuffer);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("LiDAR");
        NetworkTableEntry mmDistance = builder.getEntry("Distance");
        NetworkTableEntry standardDeviation = builder.getEntry("Standard Deviation");
        NetworkTableEntry isValidEntry = builder.getEntry("isValid");
        builder.setUpdateTable(() -> {
            mmDistance.setDouble(getDistance(true));
            isValidEntry.setBoolean(isValid);
            standardDeviation.setDouble(standardDeviationValue);
        });
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        if (pidSource != PIDSourceType.kDisplacement) {
            throw new IllegalArgumentException("Only displacement is supported");
        }
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return getDistance(true);
    }
}