// Package tire implements the classic empirical tire longitudinal-force
// model (Pacejka "Magic Formula", pure longitudinal slip, no offsets).
//
// The package is split by responsibility:
//
//	slip.go     — slip-ratio stiffness/curvature correction
//	formula.go  — main curve evaluation (single point)
//	peak.go     — peak amplitude D <-> vertical load conversion (single definition)
//	sweep.go    — slip-interval sweep / peak search and curve sampling
//	validate.go — input validation
//	demo.go     — built-in passenger-car demo parameter set
//
// All functions are pure: no shared mutable state, so concurrent callers
// (one per simulation loop) are fully isolated from each other.
package tire

// Coefficients carries the Magic Formula shape coefficients for pure
// longitudinal slip.
//
// The peak amplitude D is either given directly (Peak) or derived from a
// friction coefficient (Friction) as D = Friction * verticalLoad. Exactly
// one of the two must be set; the conversion itself lives only in
// PeakAmplitude (peak.go).
type Coefficients struct {
	Stiffness float64  `json:"stiffness"`          // B — stiffness factor
	Shape     float64  `json:"shape"`              // C — shape factor
	Curvature float64  `json:"curvature"`          // E — curvature factor
	Peak      *float64 `json:"peak,omitempty"`     // D — peak amplitude, given directly
	Friction  *float64 `json:"friction,omitempty"` // mu — D = mu * verticalLoad
}
