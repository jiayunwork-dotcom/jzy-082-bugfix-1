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

// When the requested window does not contain the curve peak, the reported
// peak slip must be the window boundary carrying the largest |Fx| — never a
// slip outside [min, max]. These cases lock the regression where golden-
// section refinement stepped past the window edge.
func TestSweepPeakRespectsWindowBounds(t *testing.T) {
	c, fz := demoLike(t)
	cases := []struct {
		name      string
		spec      SweepSpec
		wantSlip  float64
		wantForce float64 // ballpark value (single-point evaluation gives the exact check)
	}{
		{
			name:      "peak_at_upper_endpoint",
			spec:      SweepSpec{Min: 0.02, Max: 0.12, Points: 11},
			wantSlip:  0.12,
			wantForce: 3925.36,
		},
		{
			// Dense grid: previously the refinement overshot by ~one step.
			name:      "peak_at_upper_endpoint_dense_grid",
			spec:      SweepSpec{Min: 0.02, Max: 0.12, Points: 2000},
			wantSlip:  0.12,
			wantForce: 3925.36,
		},
		{
			name:      "peak_at_lower_endpoint_behind_curve_peak",
			spec:      SweepSpec{Min: 0.3, Max: 0.8, Points: 11},
			wantSlip:  0.3,
			wantForce: 3943.01,
		},
		{
			name:      "peak_at_lower_endpoint_braking",
			spec:      SweepSpec{Min: -0.12, Max: -0.02, Points: 11},
			wantSlip:  -0.12,
			wantForce: -3925.36,
		},
		{
			name:      "peak_at_lower_endpoint_braking_dense_grid",
			spec:      SweepSpec{Min: -0.12, Max: -0.02, Points: 2000},
			wantSlip:  -0.12,
			wantForce: -3925.36,
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			res, err := SweepPeak(c, fz, tc.spec)
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if res.PeakSlip < tc.spec.Min || res.PeakSlip > tc.spec.Max {
				t.Fatalf("peak slip %v outside requested window [%v, %v]",
					res.PeakSlip, tc.spec.Min, tc.spec.Max)
			}
			if res.PeakSlip != tc.wantSlip {
				t.Fatalf("peak slip %v, want exact endpoint %v", res.PeakSlip, tc.wantSlip)
			}
			if !almostEqual(res.PeakForce, tc.wantForce, 1e-4) {
				t.Fatalf("peak force %v, want ~%v", res.PeakForce, tc.wantForce)
			}

			// The reported force must equal the single-point formula at the
			// reported slip, exactly.
			force, _, err := LongitudinalForce(c, fz, res.PeakSlip)
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if force != res.PeakForce {
				t.Fatalf("single-point re-evaluation %v != reported peak force %v",
					force, res.PeakForce)
			}

			// It must also be the maximum |Fx| over the whole window.
			const dense = 100001
			maxAbs := -1.0
			for i := 0; i < dense; i++ {
				k := tc.spec.Min + (tc.spec.Max-tc.spec.Min)*float64(i)/float64(dense-1)
				if a := math.Abs(Evaluate(c, res.PeakAmplitude, k)); a > maxAbs {
					maxAbs = a
				}
			}
			if math.Abs(res.PeakForce) < maxAbs*(1-1e-9) {
				t.Fatalf("reported |force| %v is below window maximum %v",
					math.Abs(res.PeakForce), maxAbs)
			}
			if math.Signbit(res.PeakForce) != math.Signbit(tc.wantForce) {
				t.Fatalf("force %v has the wrong sign for endpoint slip %v",
					res.PeakForce, tc.wantSlip)
			}
		})
	}
}

// A direct peak amplitude D (instead of mu * Fz) must obey the same window
// bounds — the refinement fix lives after D is resolved.
func TestSweepPeakRespectsWindowBoundsDirectAmplitude(t *testing.T) {
	d := 5000.0
	c := Coefficients{Stiffness: 12, Shape: 1.65, Curvature: 0.9, Peak: &d}
	spec := SweepSpec{Min: 0.02, Max: 0.12, Points: 11}
	res, err := SweepPeak(c, 4500, spec)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if res.PeakSlip != 0.12 {
		t.Fatalf("peak slip %v, want 0.12", res.PeakSlip)
	}
	if res.PeakSlip < spec.Min || res.PeakSlip > spec.Max {
		t.Fatalf("peak slip %v outside window", res.PeakSlip)
	}
	force, _, err := LongitudinalForce(c, 4500, res.PeakSlip)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if force != res.PeakForce {
		t.Fatalf("single-point force %v != reported peak force %v", force, res.PeakForce)
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
