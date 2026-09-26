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

// A narrow window entirely on the rising flank of the curve must report
// the upper endpoint as the peak: the true curve peak (~0.18 for the demo
// parameters) lies outside the window, so the search must not chase it.
func TestSweepWindowBeforePeakReportsUpperEndpoint(t *testing.T) {
	c, fz := demoLike(t)
	spec := SweepSpec{Min: 0.02, Max: 0.12, Points: 11}
	res, err := SweepPeak(c, fz, spec)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if res.PeakSlip != spec.Max {
		t.Fatalf("peak slip %v, want upper endpoint %v", res.PeakSlip, spec.Max)
	}
	want := Evaluate(c, res.PeakAmplitude, spec.Max)
	if res.PeakForce != want {
		t.Fatalf("peak force %v, want force at upper endpoint %v", res.PeakForce, want)
	}
	if wantForce := 3925.3606934482364; !almostEqual(res.PeakForce, wantForce, 1e-9) {
		t.Fatalf("peak force %v, want ~%v", res.PeakForce, wantForce)
	}
	// Re-evaluation through the single-point formula must agree exactly.
	f, _, err := LongitudinalForce(c, fz, res.PeakSlip)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if f != res.PeakForce {
		t.Fatalf("single-point re-evaluation %v != sweep peak force %v", f, res.PeakForce)
	}
}

// The braking-side mirror: a negative window on the rising (|Fx|) flank
// must report the lower (most negative) endpoint, with negative force.
func TestSweepWindowBeforePeakReportsLowerEndpointBraking(t *testing.T) {
	c, fz := demoLike(t)
	spec := SweepSpec{Min: -0.12, Max: -0.02, Points: 11}
	res, err := SweepPeak(c, fz, spec)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if res.PeakSlip != spec.Min {
		t.Fatalf("peak slip %v, want lower endpoint %v", res.PeakSlip, spec.Min)
	}
	want := Evaluate(c, res.PeakAmplitude, spec.Min)
	if res.PeakForce != want {
		t.Fatalf("peak force %v, want force at lower endpoint %v", res.PeakForce, want)
	}
	if wantForce := -3925.3606934482364; !almostEqual(res.PeakForce, wantForce, 1e-9) {
		t.Fatalf("peak force %v, want ~%v", res.PeakForce, wantForce)
	}
	if res.PeakForce >= 0 {
		t.Fatalf("braking-side peak force %v must be negative", res.PeakForce)
	}
}

// A window entirely on the falling flank past the peak must report the
// lower endpoint.
func TestSweepWindowAfterPeakReportsLowerEndpoint(t *testing.T) {
	c, fz := demoLike(t)
	spec := SweepSpec{Min: 0.3, Max: 0.8, Points: 11}
	res, err := SweepPeak(c, fz, spec)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if res.PeakSlip != spec.Min {
		t.Fatalf("peak slip %v, want lower endpoint %v", res.PeakSlip, spec.Min)
	}
	want := Evaluate(c, res.PeakAmplitude, spec.Min)
	if res.PeakForce != want {
		t.Fatalf("peak force %v, want force at lower endpoint %v", res.PeakForce, want)
	}
	if wantForce := 3943.00966256311; !almostEqual(res.PeakForce, wantForce, 1e-9) {
		t.Fatalf("peak force %v, want ~%v", res.PeakForce, wantForce)
	}
}

// No matter how the window is placed, the reported peak must lie inside
// the requested interval (endpoints included) and its |Fx| must equal the
// maximum |Fx| over a dense sampling of that interval.
func TestSweepResultStaysInsideWindow(t *testing.T) {
	c, fz := demoLike(t)
	windows := []SweepSpec{
		{Min: 0.02, Max: 0.12, Points: 11},
		{Min: -0.12, Max: -0.02, Points: 11},
		{Min: 0.3, Max: 0.8, Points: 11},
		{Min: 0.02, Max: 0.12, Points: 2000},
		{Min: 0.1, Max: 0.25, Points: 11}, // window containing the true peak
		{Min: -0.5, Max: 0.5, Points: 50},
	}
	for _, spec := range windows {
		res, err := SweepPeak(c, fz, spec)
		if err != nil {
			t.Fatalf("window [%v, %v]: unexpected error: %v", spec.Min, spec.Max, err)
		}
		if res.PeakSlip < spec.Min || res.PeakSlip > spec.Max {
			t.Fatalf("window [%v, %v]: peak slip %v outside interval", spec.Min, spec.Max, res.PeakSlip)
		}
		const dense = 10000
		maxAbs := -1.0
		for i := 0; i <= dense; i++ {
			k := spec.Min + (spec.Max-spec.Min)*float64(i)/float64(dense)
			a := math.Abs(Evaluate(c, res.PeakAmplitude, k))
			if a > maxAbs {
				maxAbs = a
			}
		}
		// The refined peak may exceed the dense-grid maximum by a hair; the
		// dense grid must never beat the reported result.
		if math.Abs(res.PeakForce) < maxAbs-1e-9 {
			t.Fatalf("window [%v, %v]: peak force |%v| below dense-grid max %v",
				spec.Min, spec.Max, res.PeakForce, maxAbs)
		}
	}
}

// The same windowing guarantee must hold when the peak amplitude D is
// given directly instead of being derived from friction and load.
func TestSweepWindowedDirectPeakReportsEndpoint(t *testing.T) {
	d := 5000.0
	c := Coefficients{Stiffness: 10, Shape: 1.9, Curvature: 0.97, Peak: &d}
	res, err := SweepPeak(c, 4500, SweepSpec{Min: 0.02, Max: 0.12, Points: 11})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if res.PeakSlip != 0.12 {
		t.Fatalf("peak slip %v, want 0.12", res.PeakSlip)
	}
	if res.PeakAmplitude != d {
		t.Fatalf("peak amplitude %v, want %v", res.PeakAmplitude, d)
	}
	if res.PeakForce >= d {
		t.Fatalf("windowed peak force %v must stay below amplitude %v", res.PeakForce, d)
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
