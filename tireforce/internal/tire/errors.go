package tire

import "fmt"

// ErrorKind classifies validation failures so the API layer can map them
// onto structured error responses.
type ErrorKind string

const (
	// ErrKindLoad marks an illegal vertical load (non-positive or non-finite).
	ErrKindLoad ErrorKind = "invalid_load"
	// ErrKindShape marks degenerate shape coefficients (e.g. non-positive
	// shape factor, zero stiffness, missing/illegal amplitude definition).
	ErrKindShape ErrorKind = "invalid_shape"
	// ErrKindSweep marks an illegal sweep specification.
	ErrKindSweep ErrorKind = "invalid_sweep"
)

// ValidationError is a structured input-validation failure detected before
// any computation takes place.
type ValidationError struct {
	Kind    ErrorKind
	Field   string
	Message string
}

func (e *ValidationError) Error() string {
	return fmt.Sprintf("%s: %s: %s", e.Kind, e.Field, e.Message)
}

func newValidationError(kind ErrorKind, field, format string, args ...any) *ValidationError {
	return &ValidationError{Kind: kind, Field: field, Message: fmt.Sprintf(format, args...)}
}
