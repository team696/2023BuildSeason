package frc.lib.common.narwhaldashboard;

/**
 * A callback to be used with data sent by NarwhalDashboard
 */
public interface NumericalDataCallback {
    void process(double[] data);
}