package tire

// PeakAmplitude resolves the peak amplitude D of the force–slip curve.
//
// This is THE ONLY place where D is derived from the vertical load:
//
//   - if Coefficients.Peak is set, it is used directly;
//   - otherwise D = Friction * verticalLoad.
//
// Single-point evaluation, curve sampling and the peak sweep all call this
// one function, so the "D = mu * Fz" relation is defined exactly once and
// the entry points can never silently disagree.
func PeakAmplitude(c Coefficients, verticalLoad float64) (float64, error) {
	switch {
	case c.Peak != nil:
		return *c.Peak, nil
	case c.Friction != nil:
		return *c.Friction * verticalLoad, nil
	default:
		return 0, newValidationError(ErrKindShape, "coefficients",
			`peak amplitude missing: provide either "peak" (D) or "friction" (mu)`)
	}
}
