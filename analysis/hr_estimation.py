import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, find_peaks


def bandpass_filter(signal, low=0.5, high=4.0, fs=61.0, order=4):
    """Butterworth bandpass filter, zero-phase (forward-backward)."""
    nyq = fs / 2
    b, a = butter(order, [low / nyq, high / nyq], btype='band')
    return filtfilt(b, a, signal)


def est_window_hr(signal, fs=61.0, min_hr=40, max_hr=200):
    """Estimate HR from a single filtered window using peak detection."""
    min_distance = int((60 / max_hr) * fs)
    peaks, _ = find_peaks(signal, distance=min_distance, prominence=np.std(signal) * 0.8)
    if len(peaks) < 2:
        return np.nan

    rr_intervals = np.diff(peaks) / fs
    clean_rr = rr_intervals[
        (rr_intervals > np.median(rr_intervals) * 0.75) &
        (rr_intervals < np.median(rr_intervals) * 1.25)
    ]
    if len(clean_rr) == 0:
        return np.nan

    hr = 60.0 / np.mean(clean_rr)
    return hr if min_hr <= hr <= max_hr else np.nan


def kalman_smooth(hr_values, Q=1.0, R=25.0):
    """1D Kalman filter on HR estimates (NaNs are bridged by prediction).

    Q = process noise  — increase for faster-changing HR (exercise)
    R = measurement noise — decrease to trust peak-detection estimates more
    """
    n = len(hr_values)
    x = next((v for v in hr_values if not np.isnan(v)), 75.0)
    P = 10.0
    smoothed = np.zeros(n)
    for i in range(n):
        P_pred = P + Q
        if not np.isnan(hr_values[i]):
            K = P_pred / (P_pred + R)
            x = x + K * (hr_values[i] - x)
            P = (1 - K) * P_pred
        else:
            P = P_pred   # uncertainty grows while bridging NaN
        smoothed[i] = x
    return smoothed


def interpolate_blank_ppg(clean_ppg):
    """Fill NaN gaps in a PPG array using linear interpolation."""
    indices = np.arange(len(clean_ppg))
    valid = ~np.isnan(clean_ppg)
    if valid.sum() < 2:
        return clean_ppg
    return np.interp(indices, indices[valid], clean_ppg[valid])


def calc_accel_magnitude(df):
    """Per-sample accelerometer magnitude (requires x_accel_mg/y_accel_mg/z_accel_mg columns)."""
    return np.sqrt(df['x_accel_mg']**2 + df['y_accel_mg']**2 + df['z_accel_mg']**2).values


def remove_accel_spikes(ppg_signal, accel_mag, accel_threshold=1e5):
    """Replace samples where accelerometer is saturated with NaN."""
    spike_mask = accel_mag > accel_threshold
    clean_ppg = ppg_signal.copy().astype(float)
    clean_ppg[spike_mask] = np.nan
    return clean_ppg, spike_mask


def est_hr(df, ir_col='ir', ts_col='timestamp_ms', window_sec=8, step_sec=1, fs=None):
    """Estimate heart rate at 1 Hz from an ESP32/MAX30102 PPG DataFrame.

    Parameters
    ----------
    df         : DataFrame with columns [ts_col, ir_col]
    ir_col     : IR channel column (default 'ir')
    ts_col     : timestamp column in ms (default 'timestamp_ms')
    window_sec : sliding window length in seconds (default 8)
    step_sec   : output cadence in seconds (default 1)
    fs         : sampling rate Hz; auto-detected from ts_col if None

    Returns
    -------
    DataFrame  index=timestamp_ms, column='hr_bpm'
    """
    if fs is None:
        ts = df[ts_col].values
        fs = 1000.0 / np.median(np.diff(ts))

    window_samples = int(window_sec * fs)
    step_samples   = int(step_sec * fs)
    timestamps     = df[ts_col].values
    signal         = df[ir_col].values.astype(float)

    filtered = bandpass_filter(signal, fs=fs)

    hr_estimates  = []
    hr_timestamps = []
    for start in range(0, len(filtered) - window_samples, step_samples):
        end    = start + window_samples
        hr     = est_window_hr(filtered[start:end], fs=fs)
        hr_estimates.append(hr)
        hr_timestamps.append(timestamps[end - 1])

    hr_estimates = kalman_smooth(hr_estimates).tolist()

    result = pd.DataFrame({'hr_bpm': hr_estimates}, index=hr_timestamps)
    result.index.name = ts_col
    return result


def calc_error(estimated_hr, ref_hr):
    """MAE, RMSE, bias, and R² between estimated and reference HR DataFrames.

    Both DataFrames must have index in ms and column 'hr_bpm'.
    Timestamps are aligned by rounding to the nearest second.
    """
    est = estimated_hr.copy()
    ref = ref_hr.copy()
    est.index = (estimated_hr.index / 1000).round().astype(int) * 1000
    ref.index = (ref_hr.index / 1000).round().astype(int) * 1000

    merged = est.join(ref, how='inner', lsuffix='_est', rsuffix='_ref').dropna()
    errors = merged['hr_bpm_est'] - merged['hr_bpm_ref']

    mae   = np.mean(np.abs(errors))
    rmse  = np.sqrt(np.mean(errors**2))
    me    = np.mean(errors)
    r2    = 1 - np.sum(errors**2) / np.sum((merged['hr_bpm_ref'] - merged['hr_bpm_ref'].mean())**2)

    print(f"MAE:  {mae:.2f} bpm")
    print(f"RMSE: {rmse:.2f} bpm")
    print(f"Bias: {me:.2f} bpm ({'over' if me > 0 else 'under'}estimating)")
    print(f"R²:   {r2:.3f}")
    return float(rmse)
