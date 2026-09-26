package tire

import (
	"fmt"
	"math"
)

// NormalSlipLimit is the slip-ratio magnitude beyond which a warning is
// issued. The value is still computed; the warning is purely advisory.
const NormalSlipLimit = 1.0

// ValidateLoad rejects non-positive or non-finite vertical loads.
func ValidateLoad(verticalLoad float64) error {
	if math.IsNaN(verticalLoad) || math.IsInf(verticalLoad, 0) || verticalLoad <= 0 {
		return newValidationError(ErrKindLoad, "vertical_load",
			"vertical load must be a positive finite number, got %v", verticalLoad)
	}
	return nil
}

// ValidateShape rejects degenerate coefficient sets for which the curve has
// no meaningful peak: non-positive shape factor C, zero stiffness B,
// non-finite coefficients, or an amplitude definition that is missing,
// contradictory, or non-positive.
func ValidateShape(c Coefficients) error {
	finite := map[string]float64{
		"stiffness": c.Stiffness,
		"shape":     c.Shape,
		"curvature": c.Curvature,
	}
	for field, v := range finite {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return newValidationError(ErrKindShape, field,
				"coefficient must be finite, got %v", v)
		}
	}
	if c.Shape <= 0 {
		return newValidationError(ErrKindShape, "shape",
			"shape factor C must be positive, got %v", c.Shape)
	}
	if c.Stiffness == 0 {
		return newValidationError(ErrKindShape, "stiffness",
			"stiffness factor B must be non-zero, got %v", c.Stiffness)
	}
	switch {
	case c.Peak != nil && c.Friction != nil:
		return newValidationError(ErrKindShape, "coefficients",
			`ambiguous amplitude: provide either "peak" (D) or "friction" (mu), not both`)
	case c.Peak != nil && !positiveFinite(*c.Peak):
		return newValidationError(ErrKindShape, "peak",
			"peak amplitude D must be a positive finite number, got %v", *c.Peak)
	case c.Friction != nil && !positiveFinite(*c.Friction):
		return newValidationError(ErrKindShape, "friction",
			"friction coefficient mu must be a positive finite number, got %v", *c.Friction)
	case c.Peak == nil && c.Friction == nil:
		return newValidationError(ErrKindShape, "coefficients",
			`peak amplitude missing: provide either "peak" (D) or "friction" (mu)`)
	}
	return nil
}

// Validate runs all input validation before any computation.
func Validate(c Coefficients, verticalLoad float64) error {
	if err := ValidateLoad(verticalLoad); err != nil {
		return err
	}
	return ValidateShape(c)
}

// SlipWarning returns an advisory message when the slip magnitude is
// outside the normal operating range, or "" when it is inside.
func SlipWarning(kappa float64) string {
	if math.Abs(kappa) > NormalSlipLimit {
		return fmt.Sprintf("slip |kappa|=%g exceeds the normal range |kappa| <= %g; value computed anyway",
			kappa, NormalSlipLimit)
	}
	return ""
}

func positiveFinite(v float64) bool {
	return v > 0 && !math.IsNaN(v) && !math.IsInf(v, 0)
}
