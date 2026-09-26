package api

import (
	"bytes"
	"encoding/json"
	"fmt"
	"math"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"

	"github.com/gin-gonic/gin"

	"tireforce/internal/tire"
)

func init() { gin.SetMode(gin.TestMode) }

func doRequest(t *testing.T, r http.Handler, method, path string, body any) (*httptest.ResponseRecorder, map[string]any) {
	t.Helper()
	var buf bytes.Buffer
	if body != nil {
		if err := json.NewEncoder(&buf).Encode(body); err != nil {
			t.Fatalf("encode body: %v", err)
		}
	}
	req := httptest.NewRequest(method, path, &buf)
	req.Header.Set("Content-Type", "application/json")
	rec := httptest.NewRecorder()
	r.ServeHTTP(rec, req)
	var parsed map[string]any
	if rec.Body.Len() > 0 {
		if err := json.Unmarshal(rec.Body.Bytes(), &parsed); err != nil {
			t.Fatalf("response not JSON: %v\nbody: %s", err, rec.Body.String())
		}
	}
	return rec, parsed
}

func demoPayload(load, slip float64) map[string]any {
	return map[string]any{
		"vertical_load": load,
		"slip":          slip,
		"coefficients": map[string]any{
			"stiffness": 10.0, "shape": 1.9, "curvature": 0.97, "friction": 1.0,
		},
	}
}

func TestForceEndpoint(t *testing.T) {
	r := NewRouter()
	rec, body := doRequest(t, r, http.MethodPost, "/api/v1/longitudinal/force", demoPayload(4000, 0.05))
	if rec.Code != http.StatusOK {
		t.Fatalf("status %d, body %v", rec.Code, body)
	}
	force := body["longitudinal_force"].(float64)
	amp := body["peak_amplitude"].(float64)
	if force <= 0 {
		t.Fatalf("positive slip gave non-positive force %v", force)
	}
	if amp != 4000 {
		t.Fatalf("peak amplitude %v, want 4000 (mu * Fz)", amp)
	}
	if math.Abs(force) >= amp {
		t.Fatalf("small-slip force %v not below amplitude %v", force, amp)
	}
}

func TestForceEndpointZeroSlip(t *testing.T) {
	r := NewRouter()
	_, body := doRequest(t, r, http.MethodPost, "/api/v1/longitudinal/force", demoPayload(4000, 0))
	if body["longitudinal_force"].(float64) != 0 {
		t.Fatalf("zero slip gave force %v", body["longitudinal_force"])
	}
}

func TestForceEndpointInvalidLoad(t *testing.T) {
	r := NewRouter()
	rec, body := doRequest(t, r, http.MethodPost, "/api/v1/longitudinal/force", demoPayload(-100, 0.05))
	if rec.Code != http.StatusBadRequest {
		t.Fatalf("status %d, want 400", rec.Code)
	}
	if body["error"].(map[string]any)["type"] != "invalid_load" {
		t.Fatalf("error body %v, want type invalid_load", body)
	}
}

func TestForceEndpointInvalidShape(t *testing.T) {
	r := NewRouter()
	payload := demoPayload(4000, 0.05)
	payload["coefficients"] = map[string]any{"stiffness": 10.0, "shape": -1.9, "curvature": 0.97, "friction": 1.0}
	rec, body := doRequest(t, r, http.MethodPost, "/api/v1/longitudinal/force", payload)
	if rec.Code != http.StatusBadRequest {
		t.Fatalf("status %d, want 400", rec.Code)
	}
	if body["error"].(map[string]any)["type"] != "invalid_shape" {
		t.Fatalf("error body %v, want type invalid_shape", body)
	}
}

func TestForceEndpointSlipWarning(t *testing.T) {
	r := NewRouter()
	rec, body := doRequest(t, r, http.MethodPost, "/api/v1/longitudinal/force", demoPayload(4000, 1.5))
	if rec.Code != http.StatusOK {
		t.Fatalf("status %d, want 200 (out-of-range slip still computes)", rec.Code)
	}
	if _, ok := body["warnings"]; !ok {
		t.Fatalf("expected warnings for |slip| > 1, body %v", body)
	}
}

// The peak endpoint and the force endpoint must agree: re-evaluating the
// reported peak slip through the single-point endpoint reproduces the
// reported peak force.
func TestPeakEndpointConsistentWithForce(t *testing.T) {
	r := NewRouter()
	rec, peak := doRequest(t, r, http.MethodPost, "/api/v1/longitudinal/peak", map[string]any{
		"vertical_load": 4000.0,
		"coefficients":  map[string]any{"stiffness": 10.0, "shape": 1.9, "curvature": 0.97, "friction": 1.0},
	})
	if rec.Code != http.StatusOK {
		t.Fatalf("status %d, body %v", rec.Code, peak)
	}
	peakSlip := peak["peak_slip"].(float64)
	peakForce := peak["peak_force"].(float64)

	_, single := doRequest(t, r, http.MethodPost, "/api/v1/longitudinal/force", demoPayload(4000, peakSlip))
	if single["longitudinal_force"].(float64) != peakForce {
		t.Fatalf("single-point force %v != sweep peak force %v",
			single["longitudinal_force"], peakForce)
	}
}

func TestCurveEndpoint(t *testing.T) {
	r := NewRouter()
	rec, body := doRequest(t, r, http.MethodPost, "/api/v1/longitudinal/curve", map[string]any{
		"vertical_load": 4000.0,
		"coefficients":  map[string]any{"stiffness": 10.0, "shape": 1.9, "curvature": 0.97, "friction": 1.0},
		"slips":         []float64{-0.2, 0, 0.1},
	})
	if rec.Code != http.StatusOK {
		t.Fatalf("status %d, body %v", rec.Code, body)
	}
	points := body["points"].([]any)
	if len(points) != 3 {
		t.Fatalf("got %d points, want 3", len(points))
	}
	zero := points[1].(map[string]any)
	if zero["force"].(float64) != 0 {
		t.Fatalf("curve at zero slip gave force %v", zero["force"])
	}
	neg := points[0].(map[string]any)["force"].(float64)
	pos := points[2].(map[string]any)["force"].(float64)
	if neg >= 0 || pos <= 0 {
		t.Fatalf("curve signs wrong: F(-0.2)=%v, F(0.1)=%v", neg, pos)
	}
}

func TestCurveEndpointRejectsEmptyGrid(t *testing.T) {
	r := NewRouter()
	rec, _ := doRequest(t, r, http.MethodPost, "/api/v1/longitudinal/curve", map[string]any{
		"vertical_load": 4000.0,
		"coefficients":  map[string]any{"stiffness": 10.0, "shape": 1.9, "curvature": 0.97, "friction": 1.0},
		"slips":         []float64{},
	})
	if rec.Code != http.StatusBadRequest {
		t.Fatalf("status %d, want 400", rec.Code)
	}
}

func TestDemoEndpoint(t *testing.T) {
	r := NewRouter()
	rec, body := doRequest(t, r, http.MethodGet, "/api/v1/demo", nil)
	if rec.Code != http.StatusOK {
		t.Fatalf("status %d, body %v", rec.Code, body)
	}
	sample := body["sample"].(map[string]any)
	force := sample["longitudinal_force"].(float64)
	amp := sample["peak_amplitude"].(float64)
	slip := sample["slip"].(float64)
	if math.Signbit(force) != math.Signbit(slip) {
		t.Fatalf("demo: force sign %v disagrees with slip sign %v", force, slip)
	}
	if math.Abs(force) >= amp {
		t.Fatalf("demo: small-slip force %v not below amplitude %v", force, amp)
	}
}

// Concurrent simulation clients must not leak coefficients or sweep state
// into each other: every response must match a local, isolated computation.
func TestConcurrentClientsAreIsolated(t *testing.T) {
	r := NewRouter()
	const clients = 32
	const requestsPerClient = 25

	var wg sync.WaitGroup
	errs := make(chan error, clients*requestsPerClient)
	for client := 0; client < clients; client++ {
		wg.Add(1)
		go func(client int) {
			defer wg.Done()
			// Each "simulation loop" uses its own coefficients and load.
			mu := 0.7 + 0.02*float64(client)
			load := 2500.0 + 100.0*float64(client)
			coeffs := tire.Coefficients{Stiffness: 8 + float64(client%5), Shape: 1.9, Curvature: 0.97, Friction: &mu}
			for i := 0; i < requestsPerClient; i++ {
				slip := 0.01 + 0.004*float64(i)
				rec, body := func() (*httptest.ResponseRecorder, map[string]any) {
					var buf bytes.Buffer
					_ = json.NewEncoder(&buf).Encode(map[string]any{
						"vertical_load": load,
						"slip":          slip,
						"coefficients": map[string]any{
							"stiffness": coeffs.Stiffness, "shape": coeffs.Shape,
							"curvature": coeffs.Curvature, "friction": mu,
						},
					})
					req := httptest.NewRequest(http.MethodPost, "/api/v1/longitudinal/force", &buf)
					req.Header.Set("Content-Type", "application/json")
					rec := httptest.NewRecorder()
					r.ServeHTTP(rec, req)
					var parsed map[string]any
					_ = json.Unmarshal(rec.Body.Bytes(), &parsed)
					return rec, parsed
				}()
				if rec.Code != http.StatusOK {
					errs <- fmt.Errorf("client %d: status %d, body %v", client, rec.Code, body)
					continue
				}
				want, wantAmp, err := tire.LongitudinalForce(coeffs, load, slip)
				if err != nil {
					errs <- fmt.Errorf("client %d: local eval: %w", client, err)
					continue
				}
				if got := body["longitudinal_force"].(float64); got != want {
					errs <- fmt.Errorf("client %d: force %v, want %v", client, got, want)
				}
				if got := body["peak_amplitude"].(float64); got != wantAmp {
					errs <- fmt.Errorf("client %d: amplitude %v, want %v", client, got, wantAmp)
				}
			}
		}(client)
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		t.Error(err)
	}
}
