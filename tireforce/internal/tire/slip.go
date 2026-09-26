package tire

import "math"

// CorrectedSlip applies the stiffness (B) and curvature (E) correction to
// the slip ratio before it enters the arctangent of the main curve:
//
//	x = B * kappa
//	y = x - E * (x - atan(x))
//
// The correction is odd in kappa (no horizontal shift), which is what makes
// the whole force curve point-symmetric about the origin.
func CorrectedSlip(kappa, stiffness, curvature float64) float64 {
	x := stiffness * kappa
	return x - curvature*(x-math.Atan(x))
}
