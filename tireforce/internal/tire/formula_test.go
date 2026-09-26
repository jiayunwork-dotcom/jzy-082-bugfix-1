package tire

import (
	"math"
	"testing"
)

const tol = 1e-12

func almostEqual(a, b, relTol float64) bool {
	if a == b {
		return true
	}
	return math.Abs(a-b) <= relTol*math.Max(1, math.Max(math.Abs(a), math.Abs(b)))
}

func demoLike(t *testing.T) (Coefficients, float64) {
	t.Helper()
	return DemoCoefficients(), DemoVerticalLoad
}

// Zero slip must yield exactly zero force.
func TestZeroSlipYieldsZeroForce(t *testing.T) {
	c, fz := demoLike(t)
	for _, kappa := range []float64{0, math.Copysign(0, -1)} {
		f, _, err := LongitudinalForce(c, fz, kappa)
		if err != nil {
			t.Fatalf("unexpected error: %v", err)
		}
		if f != 0 {
			t.Fatalf("F(%v) = %v, want exactly 0", kappa, f)
		}
	}
}

// Flipping the slip sign must flip the force sign with identical magnitude
// (the model has no horizontal shift).
func TestOddSymmetry(t *testing.T) {
	c, fz := demoLike(t)
	d, err := PeakAmplitude(c, fz)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	for _, kappa := range []float64{0.001, 0.01, 0.05, 0.13, 0.3, 0.77, 1.0, 1.4} {
		fp := Evaluate(c, d, kappa)
		fn := Evaluate(c, d, -kappa)
		if !almostEqual(fp, -fn, tol) {
			t.Fatalf("F(%v)=%v, F(%v)=%v: want equal magnitude, opposite sign", kappa, fp, -kappa, fn)
		}
	}
}

// With D = mu * Fz, scaling the load by k must scale the peak force by k.
func TestPeakForceScalesWithLoad(t *testing.T) {
	c, fz := demoLike(t)
	spec := SweepSpec{Min: 0, Max: 1, Points: 2000}

	base, err := SweepPeak(c, fz, spec)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	for _, k := range []float64{0.5, 1.5, 2, 3.25} {
		scaled, err := SweepPeak(c, k*fz, spec)
		if err != nil {
			t.Fatalf("unexpected error: %v", err)
		}
		if !almostEqual(scaled.PeakForce, k*base.PeakForce, 1e-9) {
			t.Fatalf("load x%v: peak force %v, want %v", k, scaled.PeakForce, k*base.PeakForce)
		}
		if !almostEqual(scaled.PeakSlip, base.PeakSlip, 1e-9) {
			t.Fatalf("load x%v: peak slip %v moved from %v", k, scaled.PeakSlip, base.PeakSlip)
		}
		if !almostEqual(scaled.PeakAmplitude, k*base.PeakAmplitude, 1e-12) {
			t.Fatalf("load x%v: peak amplitude %v, want %v", k, scaled.PeakAmplitude, k*base.PeakAmplitude)
		}
	}
}

// For standard shapes, |Fx| never exceeds the peak amplitude D.
func TestForceNeverExceedsPeakAmplitude(t *testing.T) {
	sets := []Coefficients{
		DemoCoefficients(),
		{Stiffness: 12, Shape: 1.65, Curvature: 0.9, Peak: ptr(5000)},
		{Stiffness: 6, Shape: 2.2, Curvature: 1.0, Peak: ptr(3200)},
		{Stiffness: 18, Shape: 1.3, Curvature: -0.5, Peak: ptr(4100)},
	}
	for i, c := range sets {
		d, err := PeakAmplitude(c, DemoVerticalLoad)
		if err != nil {
			t.Fatalf("set %d: unexpected error: %v", i, err)
		}
		for kappa := -1.5; kappa <= 1.5+1e-9; kappa += 0.001 {
			f := Evaluate(c, d, kappa)
			if math.Abs(f) > d*(1+1e-9) {
				t.Fatalf("set %d: |F(%v)| = %v exceeds D = %v", i, kappa, math.Abs(f), d)
			}
		}
	}
}

// Demo set self-check: at small slip the force follows the slip direction
// and stays below the peak amplitude.
func TestDemoSmallSlipDirection(t *testing.T) {
	c, fz := demoLike(t)
	d, err := PeakAmplitude(c, fz)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	for _, kappa := range []float64{0.005, 0.01, 0.05, 0.1} {
		fp := Evaluate(c, d, kappa)
		fn := Evaluate(c, d, -kappa)
		if fp <= 0 || fn >= 0 {
			t.Fatalf("small-slip direction wrong: F(%v)=%v, F(%v)=%v", kappa, fp, -kappa, fn)
		}
		if math.Abs(fp) >= d || math.Abs(fn) >= d {
			t.Fatalf("small-slip force |%v| not below peak amplitude %v", fp, d)
		}
	}
}

// The friction-form amplitude must be exactly mu * Fz — the relation is
// defined once in PeakAmplitude and shared by every entry point.
func TestPeakAmplitudeSingleDefinition(t *testing.T) {
	mu := 0.95
	c := Coefficients{Stiffness: 10, Shape: 1.9, Curvature: 0.97, Friction: &mu}
	fz := 4321.0
	want := mu * fz

	if got, err := PeakAmplitude(c, fz); err != nil || got != want {
		t.Fatalf("PeakAmplitude = %v, %v; want %v", got, err, want)
	}
	_, dPoint, err := LongitudinalForce(c, fz, 0.1)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if dPoint != want {
		t.Fatalf("single-point amplitude = %v, want %v", dPoint, want)
	}
	res, err := SweepPeak(c, fz, SweepSpec{Min: 0, Max: 1, Points: 500})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if res.PeakAmplitude != want {
		t.Fatalf("sweep amplitude = %v, want %v", res.PeakAmplitude, want)
	}
	_, dCurve, err := SampleCurve(c, fz, []float64{0.1})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if dCurve != want {
		t.Fatalf("curve amplitude = %v, want %v", dCurve, want)
	}
}

func ptr(v float64) *float64 { return &v }
