"""Sampling-artefact criterion for the period-2 (Nyquist) component of the measured joint velocity.

Per joint, from qd (mean removed) sampled at fs, Nyquist fN = fs / 2:
  nyquist_ratio  periodogram of the Hann-windowed qd: mean power in [NYQUIST_BAND x fN]
                 divided by mean power in [REFERENCE_BAND x fN]
A trial is invalid when any joint has nyquist_ratio > NYQUIST_RATIO_MAX.

Information only (not part of the criterion):
  artefact_amplitude_rad     peak |qd| of the band >= REFERENCE_BAND[1] x fN, divided by 2 pi fN
  artefact_fraction_of_rms   artefact_amplitude_rad / RMS tracking error of the same joint
"""
from __future__ import annotations

import numpy as np

NYQUIST_RATIO_MAX = 8.5
NYQUIST_BAND = (0.96, 1.0)
REFERENCE_BAND = (0.7, 0.9)
MIN_SAMPLES = 200
METHOD = {
    'artefact_psd_method': 'periodogram |rfft(hann * (qd - mean))|^2',
    'artefact_window': 'hann',
    'artefact_nyquist_band_fraction': list(NYQUIST_BAND),
    'artefact_reference_band_fraction': list(REFERENCE_BAND),
    'artefact_nyquist_ratio_max': NYQUIST_RATIO_MAX,
}


def nyquist_ratio(qd, rate_hz: float) -> np.ndarray:
    x = np.asarray(qd, dtype=float)
    x = x - x.mean(axis=0)
    f_n = rate_hz / 2.0
    fr = np.fft.rfftfreq(x.shape[0], 1.0 / rate_hz)
    psd = np.abs(np.fft.rfft(x * np.hanning(x.shape[0])[:, None], axis=0)) ** 2
    top = psd[(fr >= NYQUIST_BAND[0] * f_n) & (fr <= NYQUIST_BAND[1] * f_n)].mean(axis=0)
    ref = psd[(fr >= REFERENCE_BAND[0] * f_n) & (fr <= REFERENCE_BAND[1] * f_n)].mean(axis=0)
    return np.divide(top, ref, out=np.zeros_like(top), where=ref > 0)


def nyquist_artefact(qd, rms_per_joint, rate_hz: float) -> dict:
    """Criterion and information metrics; empty when the trace is shorter than MIN_SAMPLES."""
    x = np.asarray(qd, dtype=float)
    if x.shape[0] < MIN_SAMPLES:
        return {}
    ratio = nyquist_ratio(x, rate_hz)
    f_n = rate_hz / 2.0
    fr = np.fft.rfftfreq(x.shape[0], 1.0 / rate_hz)
    spec = np.fft.rfft(x - x.mean(axis=0), axis=0)
    spec[fr < REFERENCE_BAND[1] * f_n] = 0.0
    amp = np.abs(np.fft.irfft(spec, n=x.shape[0], axis=0)).max(axis=0) / (2.0 * np.pi * f_n)
    rms = np.asarray(rms_per_joint, dtype=float)
    frac = np.divide(amp, rms, out=np.full_like(amp, np.inf), where=rms > 0)
    return {
        **METHOD,
        'nyquist_ratio': ratio.tolist(),
        'artefact_amplitude_rad': amp.tolist(),
        'artefact_fraction_of_rms': frac.tolist(),
        'artefact_valid': bool(np.all(ratio <= NYQUIST_RATIO_MAX)),
    }
