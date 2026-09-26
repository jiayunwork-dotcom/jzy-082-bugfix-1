package tire

import (
	"errors"
	"math"
	"testing"
)

func TestValidateLoad(t *testing.T) {
	for _, fz := range []float64{0, -1, -4000, math.NaN(), math.Inf(1), math.Inf(-1)} {
		err := ValidateLoad(fz)
		if err == nil {
			t.Fatalf("load %v: expected error, got nil", fz)
		}
		var ve *ValidationError
		if !errors.As(err, &ve) || ve.Kind != ErrKindLoad {
			t.Fatalf("load %v: expected invalid_load, got %v", fz, err)
		}
	}
	if err := ValidateLoad(4000); err != nil {
		t.Fatalf("load 4000: unexpected error: %v", err)
	}
}

func TestValidateShapeDegenerate(t *testing.T) {
	ok := ptr(1.0)
	cases := map[string]Coefficients{
		"zero shape":          {Stiffness: 10, Shape: 0, Curvature: 0.97, Friction: ok},
		"negative shape":      {Stiffness: 10, Shape: -1.9, Curvature: 0.97, Friction: ok},
		"zero stiffness":      {Stiffness: 0, Shape: 1.9, Curvature: 0.97, Friction: ok},
		"nan curvature":       {Stiffness: 10, Shape: 1.9, Curvature: math.NaN(), Friction: ok},
		"missing amplitude":   {Stiffness: 10, Shape: 1.9, Curvature: 0.97},
		"ambiguous amplitude": {Stiffness: 10, Shape: 1.9, Curvature: 0.97, Peak: ptr(4000), Friction: ok},
		"negative peak":       {Stiffness: 10, Shape: 1.9, Curvature: 0.97, Peak: ptr(-1)},
		"zero friction":       {Stiffness: 10, Shape: 1.9, Curvature: 0.97, Friction: ptr(0)},
	}
	for name, c := range cases {
		err := ValidateShape(c)
		if err == nil {
			t.Fatalf("%s: expected error, got nil", name)
		}
		var ve *ValidationError
		if !errors.As(err, &ve) || ve.Kind != ErrKindShape {
			t.Fatalf("%s: expected invalid_shape, got %v", name, err)
		}
	}
	if err := ValidateShape(DemoCoefficients()); err != nil {
		t.Fatalf("demo coefficients: unexpected error: %v", err)
	}
}

func TestSlipWarning(t *testing.T) {
	for _, kappa := range []float64{-1, -0.5, 0, 0.5, 1} {
		if w := SlipWarning(kappa); w != "" {
			t.Fatalf("slip %v: unexpected warning %q", kappa, w)
		}
	}
	for _, kappa := range []float64{-1.0001, -2, 1.0001, 3} {
		if w := SlipWarning(kappa); w == "" {
			t.Fatalf("slip %v: expected warning, got none", kappa)
		}
	}
}
