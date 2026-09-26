package tire

import "math"

// Evaluate computes the longitudinal force for one slip ratio with an
// already-resolved peak amplitude D:
//
//	Fx = D * sin(C * atan(CorrectedSlip(kappa, B, E)))
//
// It assumes the coefficients have been validated and never fails on its
// own. Both the single-point API and the sweep evaluate the curve through
// this one function, so they can never drift apart.
func Evaluate(c Coefficients, peakAmplitude, kappa float64) float64 {
	return peakAmplitude * math.Sin(c.Shape*math.Atan(CorrectedSlip(kappa, c.Stiffness, c.Curvature)))
}

// LongitudinalForce validates the inputs, resolves the peak amplitude and
// evaluates a single operating point. It returns the force together with
// the peak amplitude that was actually used.
func LongitudinalForce(c Coefficients, verticalLoad, kappa float64) (force, peakAmplitude float64, err error) {
	if err := Validate(c, verticalLoad); err != nil {
		return 0, 0, err
	}
	d, err := PeakAmplitude(c, verticalLoad)
	if err != nil {
		return 0, 0, err
	}
	return Evaluate(c, d, kappa), d, nil
}
