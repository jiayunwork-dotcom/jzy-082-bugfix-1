package tire

import (
	"errors"
	"fmt"
	"math"
)

// SweepSpec describes the slip interval scanned for the curve peak.
type SweepSpec struct {
	Min    float64 `json:"min"`    // first slip ratio of the scan
	Max    float64 `json:"max"`    // last slip ratio of the scan
	Points int     `json:"points"` // number of grid points (>= 2)
}

// DefaultSweep is used when the caller does not provide a sweep spec. The
// curve is odd-symmetric, so scanning the positive half is enough to find
// the peak magnitude.
var DefaultSweep = SweepSpec{Min: 0, Max: NormalSlipLimit, Points: 2000}

// PeakResult is the outcome of a completed sweep.
type PeakResult struct {
	PeakSlip      float64 `json:"peak_slip"`
	PeakForce     float64 `json:"peak_force"`
	PeakAmplitude float64 `json:"peak_amplitude"`
}

// ErrSweepIncomplete marks a sweep that could not be carried through to
// completion. It is never accompanied by partial results.
var ErrSweepIncomplete = errors.New("sweep did not complete")

// SweepPeak scans the slip interval for the point of maximum |Fx| and
// returns it as the curve peak *within that interval*. The reported peak
// slip always lies in [spec.Min, spec.Max], endpoints included: when the
// force magnitude is monotone over the window (the true curve peak lies
// outside it), the boundary slip with the largest |Fx| is reported.
//
// The scan evaluates the very same curve as the single-point path
// (Evaluate with D from PeakAmplitude), and the reported peak force is
// re-evaluated at the reported peak slip with that same formula — so
// LongitudinalForce(c, fz, result.PeakSlip) reproduces result.PeakForce
// exactly.
//
// If the scan cannot complete (illegal spec, non-finite intermediate
// value), an error is returned and no partial result is reported.
func SweepPeak(c Coefficients, verticalLoad float64, spec SweepSpec) (PeakResult, error) {
	if err := Validate(c, verticalLoad); err != nil {
		return PeakResult{}, err
	}
	if err := validateSweepSpec(spec); err != nil {
		return PeakResult{}, err
	}
	d, err := PeakAmplitude(c, verticalLoad)
	if err != nil {
		return PeakResult{}, err
	}

	absForce := func(kappa float64) (float64, error) {
		f := Evaluate(c, d, kappa)
		if math.IsNaN(f) || math.IsInf(f, 0) {
			return 0, fmt.Errorf("%w: non-finite force at slip %g", ErrSweepIncomplete, kappa)
		}
		return math.Abs(f), nil
	}

	// Coarse uniform scan. The interval boundaries are seeded as explicit
	// candidates first (using the caller's exact endpoint values), so that a
	// monotone window reports the exact boundary rather than a grid-point
	// rounding of it.
	n := spec.Points
	step := (spec.Max - spec.Min) / float64(n-1)
	bestSlip, bestAbs := math.NaN(), -1.0
	consider := func(kappa float64) error {
		a, err := absForce(kappa)
		if err != nil {
			return err
		}
		if a > bestAbs {
			bestAbs, bestSlip = a, kappa
		}
		return nil
	}
	if err := consider(spec.Min); err != nil {
		return PeakResult{}, err
	}
	if err := consider(spec.Max); err != nil {
		return PeakResult{}, err
	}
	for i := 1; i < n-1; i++ {
		if err := consider(spec.Min + float64(i)*step); err != nil {
			return PeakResult{}, err
		}
	}

	// Golden-section refinement on the bracket around the best grid point.
	// The bracket is clamped to the requested interval: refinement must
	// never report a slip outside the caller's window. The refined point is
	// adopted only when it is strictly better than the best sampled point,
	// so an endpoint maximum keeps the exact boundary slip.
	lo := math.Max(spec.Min, bestSlip-step)
	hi := math.Min(spec.Max, bestSlip+step)
	if hi > lo {
		refined, err := goldenSectionMax(func(k float64) float64 {
			a, err := absForce(k)
			if err != nil {
				return math.NaN()
			}
			return a
		}, lo, hi)
		if err != nil {
			return PeakResult{}, err
		}
		if a, err := absForce(refined); err != nil {
			return PeakResult{}, err
		} else if a > bestAbs {
			bestAbs, bestSlip = a, refined
		}
	}

	// Defensive guarantee: the reported peak slip is always inside the
	// requested interval (endpoints included).
	bestSlip = math.Min(spec.Max, math.Max(spec.Min, bestSlip))
	// Final re-evaluation with the single-point formula: the reported peak
	// force is, by construction, exactly the single-point result at the
	// reported peak slip.
	peakForce := Evaluate(c, d, bestSlip)
	if math.IsNaN(peakForce) || math.IsInf(peakForce, 0) {
		return PeakResult{}, fmt.Errorf("%w: non-finite force at refined peak slip %g", ErrSweepIncomplete, bestSlip)
	}
	return PeakResult{PeakSlip: bestSlip, PeakForce: peakForce, PeakAmplitude: d}, nil
}

// CurvePoint is one sampled point of the force–slip curve.
type CurvePoint struct {
	Slip  float64 `json:"slip"`
	Force float64 `json:"force"`
}

// SampleCurve evaluates the curve on a caller-provided slip grid and
// returns the sampled points together with the peak amplitude used. If any
// point cannot be evaluated to a finite value, the whole curve is rejected
// — a partial curve is never returned.
func SampleCurve(c Coefficients, verticalLoad float64, slips []float64) ([]CurvePoint, float64, error) {
	if err := Validate(c, verticalLoad); err != nil {
		return nil, 0, err
	}
	if len(slips) == 0 {
		return nil, 0, errors.New("slip grid must not be empty")
	}
	d, err := PeakAmplitude(c, verticalLoad)
	if err != nil {
		return nil, 0, err
	}
	points := make([]CurvePoint, len(slips))
	for i, kappa := range slips {
		f := Evaluate(c, d, kappa)
		if math.IsNaN(f) || math.IsInf(f, 0) {
			return nil, 0, fmt.Errorf("non-finite force at slip grid index %d (slip=%g)", i, kappa)
		}
		points[i] = CurvePoint{Slip: kappa, Force: f}
	}
	return points, d, nil
}

func validateSweepSpec(spec SweepSpec) error {
	if math.IsNaN(spec.Min) || math.IsInf(spec.Min, 0) ||
		math.IsNaN(spec.Max) || math.IsInf(spec.Max, 0) {
		return newValidationError(ErrKindSweep, "sweep",
			"sweep bounds must be finite, got [%v, %v]", spec.Min, spec.Max)
	}
	if spec.Min >= spec.Max {
		return newValidationError(ErrKindSweep, "sweep",
			"sweep requires min < max, got min=%v max=%v", spec.Min, spec.Max)
	}
	if spec.Points < 2 {
		return newValidationError(ErrKindSweep, "sweep.points",
			"sweep needs at least 2 grid points, got %d", spec.Points)
	}
	return nil
}

// goldenSectionMax maximizes g on [a, b] assuming g is unimodal there. A
// NaN from g aborts the refinement with an error.
func goldenSectionMax(g func(float64) float64, a, b float64) (float64, error) {
	const (
		invPhi    = 0.6180339887498949 // (sqrt(5)-1)/2
		tolerance = 1e-12
	)
	c := b - invPhi*(b-a)
	d := a + invPhi*(b-a)
	for math.Abs(b-a) > tolerance {
		gc, gd := g(c), g(d)
		if math.IsNaN(gc) || math.IsNaN(gd) {
			return 0, fmt.Errorf("%w: non-finite force during peak refinement", ErrSweepIncomplete)
		}
		if gc < gd {
			a = c
			c = d
			d = a + invPhi*(b-a)
		} else {
			b = d
			d = c
			c = b - invPhi*(b-a)
		}
	}
	return (a + b) / 2, nil
}
