// Package api exposes the tire longitudinal-force model over HTTP (JSON
// in, JSON out). Handlers keep every intermediate value in request-scoped
// variables and the tire package holds no mutable state, so concurrent
// simulation clients never see each other's coefficients or sweep state.
package api

import (
	"errors"
	"fmt"
	"math"
	"net/http"

	"github.com/gin-gonic/gin"

	"tireforce/internal/tire"
)

// maxCurvePoints caps the slip grid accepted by the curve endpoint.
const maxCurvePoints = 100000

type errorBody struct {
	Type    string `json:"type"`
	Field   string `json:"field,omitempty"`
	Message string `json:"message"`
}

func writeError(c *gin.Context, status int, typ, field, msg string) {
	c.JSON(status, gin.H{"error": errorBody{Type: typ, Field: field, Message: msg}})
}

// writeTireError maps model-layer errors onto structured HTTP responses:
// validation failures are 400s, an incomplete sweep is a 500 and never
// carries partial results.
func writeTireError(c *gin.Context, err error) {
	var ve *tire.ValidationError
	if errors.As(err, &ve) {
		writeError(c, http.StatusBadRequest, string(ve.Kind), ve.Field, ve.Message)
		return
	}
	if errors.Is(err, tire.ErrSweepIncomplete) {
		writeError(c, http.StatusInternalServerError, "sweep_failed", "", err.Error())
		return
	}
	writeError(c, http.StatusInternalServerError, "computation_failed", "", err.Error())
}

type forceRequest struct {
	VerticalLoad float64           `json:"vertical_load"`
	Slip         float64           `json:"slip"`
	Coefficients tire.Coefficients `json:"coefficients"`
}

type forceResponse struct {
	Slip              float64  `json:"slip"`
	LongitudinalForce float64  `json:"longitudinal_force"`
	PeakAmplitude     float64  `json:"peak_amplitude"`
	Warnings          []string `json:"warnings,omitempty"`
}

// handleForce evaluates a single operating point.
func handleForce(c *gin.Context) {
	var req forceRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		writeError(c, http.StatusBadRequest, "bad_request", "", "invalid JSON body: "+err.Error())
		return
	}
	force, amplitude, err := tire.LongitudinalForce(req.Coefficients, req.VerticalLoad, req.Slip)
	if err != nil {
		writeTireError(c, err)
		return
	}
	if math.IsNaN(force) || math.IsInf(force, 0) {
		writeError(c, http.StatusUnprocessableEntity, "computation_failed", "slip",
			"force evaluation did not produce a finite result")
		return
	}
	resp := forceResponse{
		Slip:              req.Slip,
		LongitudinalForce: force,
		PeakAmplitude:     amplitude,
	}
	if w := tire.SlipWarning(req.Slip); w != "" {
		resp.Warnings = []string{w}
	}
	c.JSON(http.StatusOK, resp)
}

type peakRequest struct {
	VerticalLoad float64           `json:"vertical_load"`
	Coefficients tire.Coefficients `json:"coefficients"`
	Sweep        *tire.SweepSpec   `json:"sweep,omitempty"`
}

type peakResponse struct {
	PeakSlip      float64        `json:"peak_slip"`
	PeakForce     float64        `json:"peak_force"`
	PeakAmplitude float64        `json:"peak_amplitude"`
	Sweep         tire.SweepSpec `json:"sweep"`
}

// handlePeak sweeps a slip interval and reports the curve peak.
func handlePeak(c *gin.Context) {
	var req peakRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		writeError(c, http.StatusBadRequest, "bad_request", "", "invalid JSON body: "+err.Error())
		return
	}
	spec := tire.DefaultSweep
	if req.Sweep != nil {
		spec = *req.Sweep
		if spec.Points == 0 {
			spec.Points = tire.DefaultSweep.Points
		}
	}
	res, err := tire.SweepPeak(req.Coefficients, req.VerticalLoad, spec)
	if err != nil {
		writeTireError(c, err)
		return
	}
	c.JSON(http.StatusOK, peakResponse{
		PeakSlip:      res.PeakSlip,
		PeakForce:     res.PeakForce,
		PeakAmplitude: res.PeakAmplitude,
		Sweep:         spec,
	})
}

type curveRequest struct {
	VerticalLoad float64           `json:"vertical_load"`
	Coefficients tire.Coefficients `json:"coefficients"`
	Slips        []float64         `json:"slips"`
}

type curveResponse struct {
	PeakAmplitude float64           `json:"peak_amplitude"`
	Points        []tire.CurvePoint `json:"points"`
	Warnings      []string          `json:"warnings,omitempty"`
}

// handleCurve samples the force–slip curve on a caller-provided slip grid.
func handleCurve(c *gin.Context) {
	var req curveRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		writeError(c, http.StatusBadRequest, "bad_request", "", "invalid JSON body: "+err.Error())
		return
	}
	if len(req.Slips) == 0 {
		writeError(c, http.StatusBadRequest, "bad_request", "slips", "slip grid must not be empty")
		return
	}
	if len(req.Slips) > maxCurvePoints {
		writeError(c, http.StatusBadRequest, "bad_request", "slips",
			"slip grid too large")
		return
	}
	points, amplitude, err := tire.SampleCurve(req.Coefficients, req.VerticalLoad, req.Slips)
	if err != nil {
		writeTireError(c, err)
		return
	}
	resp := curveResponse{PeakAmplitude: amplitude, Points: points}
	outOfRange := 0
	for _, k := range req.Slips {
		if tire.SlipWarning(k) != "" {
			outOfRange++
		}
	}
	if outOfRange > 0 {
		resp.Warnings = []string{fmt.Sprintf(
			"%d slip value(s) exceed the normal range |kappa| <= %g; values computed anyway",
			outOfRange, tire.NormalSlipLimit)}
	}
	c.JSON(http.StatusOK, resp)
}

// handleDemo returns the built-in passenger-car demo parameter set plus a
// sample evaluation, for quick self-checks.
func handleDemo(c *gin.Context) {
	coeffs := tire.DemoCoefficients()
	const sampleSlip = 0.05
	force, amplitude, err := tire.LongitudinalForce(coeffs, tire.DemoVerticalLoad, sampleSlip)
	if err != nil {
		writeTireError(c, err)
		return
	}
	c.JSON(http.StatusOK, gin.H{
		"description":   "built-in passenger-car-scale demo parameter set (D = friction * vertical_load)",
		"coefficients":  coeffs,
		"vertical_load": tire.DemoVerticalLoad,
		"sample": gin.H{
			"slip":               sampleSlip,
			"longitudinal_force": force,
			"peak_amplitude":     amplitude,
		},
	})
}

// NewRouter builds the HTTP router.
func NewRouter() *gin.Engine {
	r := gin.New()
	r.Use(gin.Logger(), gin.Recovery())
	_ = r.SetTrustedProxies(nil)

	r.GET("/healthz", func(c *gin.Context) { c.JSON(http.StatusOK, gin.H{"status": "ok"}) })

	v1 := r.Group("/api/v1")
	v1.POST("/longitudinal/force", handleForce)
	v1.POST("/longitudinal/peak", handlePeak)
	v1.POST("/longitudinal/curve", handleCurve)
	v1.GET("/demo", handleDemo)
	return r
}
