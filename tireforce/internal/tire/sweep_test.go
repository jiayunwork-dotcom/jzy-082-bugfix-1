package tire

import (
	"errors"
	"math"
	"testing"
)

// The peak reported by the sweep must reproduce exactly when re-evaluated
// with the single-point formula at the reported peak slip.
func TestSweepPeakMatchesSinglePoint(t *testing.T) {
	sets := []struct {
		name string
		c    Coefficients
		fz   float64
	}{
		{"demo-friction", DemoCoefficients(), DemoVerticalLoad},
		{"direct-peak", Coefficients{Stiffness: 12, Shape: 1.65, Curvature: 0.9, Peak: ptr(5000)}, 4500},
	}
	for _, tc := range sets {
		t.Run(tc.name, func(t *testing.T) {
			res, err := SweepPeak(tc.c, tc.fz, SweepSpec{Min: 0, Max: 1, Points: 2000})
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			force, amplitude, err := LongitudinalForce(tc.c, tc.fz, res.PeakSlip)
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if force != res.PeakForce {
				t.Fatalf("single-point re-evaluation %v != sweep peak force %v", force, res.PeakForce)
			}
			if amplitude != res.PeakAmplitude {
				t.Fatalf("single-point amplitude %v != sweep amplitude %v", amplitude, res.PeakAmplitude)
			}
		})
	}
}

// For a standard passenger-car shape the sweep must find a peak force
// essentially equal to the peak amplitude D.
func TestSweepPeakApproachesAmplitude(t *testing.T) {
	c, fz := demoLike(t)
	res, err := SweepPeak(c, fz, DefaultSweep)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if res.PeakForce <= 0.999*res.PeakAmplitude || res.PeakForce > res.PeakAmplitude*(1+1e-9) {
		t.Fatalf("peak force %v not within tolerance of amplitude %v", res.PeakForce, res.PeakAmplitude)
	}
	if res.PeakSlip <= 0 || res.PeakSlip > 1 {
		t.Fatalf("peak slip %v outside expected range (0, 1]", res.PeakSlip)
	}
}

// The sweep is symmetric in magnitude: scanning the negative half must
// find the mirrored peak.
func TestSweepNegativeSideMirrors(t *testing.T) {
	c, fz := demoLike(t)
	pos, err := SweepPeak(c, fz, SweepSpec{Min: 0, Max: 1, Points: 1000})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	neg, err := SweepPeak(c, fz, SweepSpec{Min: -1, Max: 0, Points: 1000})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !almostEqual(neg.PeakForce, -pos.PeakForce, 1e-9) {
		t.Fatalf("negative-side peak force %v, want %v", neg.PeakForce, -pos.PeakForce)
	}
	// The refined peak slip carries float-level search noise near the flat
	// peak, so mirror agreement is checked to 1e-6 rather than exactly.
	if !almostEqual(neg.PeakSlip, -pos.PeakSlip, 1e-6) {
		t.Fatalf("negative-side peak slip %v, want %v", neg.PeakSlip, -pos.PeakSlip)
	}
}

func TestSweepRejectsInvalidSpec(t *testing.T) {
	c, fz := demoLike(t)
	for _, spec := range []SweepSpec{
		{Min: 1, Max: 0, Points: 100}, // min >= max
		{Min: 0, Max: 0, Points: 100}, // degenerate interval
		{Min: 0, Max: 1, Points: 1},   // too few points
		{Min: 0, Max: 1, Points: 0},   // no points
		{Min: math.NaN(), Max: 1, Points: 100},
		{Min: 0, Max: math.Inf(1), Points: 100},
	} {
		if _, err := SweepPeak(c, fz, spec); err == nil {
			t.Fatalf("spec %+v: expected error, got nil", spec)
		} else {
			var ve *ValidationError
			if !errors.As(err, &ve) || ve.Kind != ErrKindSweep {
				t.Fatalf("spec %+v: expected invalid_sweep error, got %v", spec, err)
			}
		}
	}
}

func TestSweepRejectsInvalidInput(t *testing.T) {
	c, _ := demoLike(t)
	spec := DefaultSweep

	// Non-positive load.
	if _, err := SweepPeak(c, 0, spec); err == nil {
		t.Fatal("zero load: expected error, got nil")
	} else {
		var ve *ValidationError
		if !errors.As(err, &ve) || ve.Kind != ErrKindLoad {
			t.Fatalf("zero load: expected invalid_load, got %v", err)
		}
	}

	// Degenerate shape.
	bad := Coefficients{Stiffness: 0, Shape: 1.9, Curvature: 0.97, Peak: ptr(4000)}
	if _, err := SweepPeak(bad, DemoVerticalLoad, spec); err == nil {
		t.Fatal("zero stiffness: expected error, got nil")
	} else {
		var ve *ValidationError
		if !errors.As(err, &ve) || ve.Kind != ErrKindShape {
			t.Fatalf("zero stiffness: expected invalid_shape, got %v", err)
		}
	}
}

func TestSampleCurveMatchesSinglePoint(t *testing.T) {
	c, fz := demoLike(t)
	slips := []float64{-0.3, -0.1, 0, 0.07, 0.25, 1.2}
	points, amplitude, err := SampleCurve(c, fz, slips)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if len(points) != len(slips) {
		t.Fatalf("got %d points, want %d", len(points), len(slips))
	}
	for i, p := range points {
		if p.Slip != slips[i] {
			t.Fatalf("point %d: slip %v, want %v", i, p.Slip, slips[i])
		}
		f, _, err := LongitudinalForce(c, fz, p.Slip)
		if err != nil {
			t.Fatalf("unexpected error: %v", err)
		}
		if f != p.Force {
			t.Fatalf("point %d: curve force %v != single-point force %v", i, p.Force, f)
		}
	}
	if amplitude <= 0 {
		t.Fatalf("curve amplitude %v, want positive", amplitude)
	}
}

func TestSampleCurveRejectsEmptyGrid(t *testing.T) {
	c, fz := demoLike(t)
	if _, _, err := SampleCurve(c, fz, nil); err == nil {
		t.Fatal("empty grid: expected error, got nil")
	}
}
