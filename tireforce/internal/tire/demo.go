package tire

// DemoVerticalLoad is a typical passenger-car corner load in newtons.
const DemoVerticalLoad = 4000.0

// DemoCoefficients returns a built-in passenger-car-scale parameter set for
// quick self-checks. With these coefficients the force at small slip has
// the same sign as the slip and stays well below the peak amplitude
// D = mu * Fz = 1.0 * 4000 N.
func DemoCoefficients() Coefficients {
	mu := 1.0
	return Coefficients{
		Stiffness: 10.0, // B
		Shape:     1.9,  // C
		Curvature: 0.97, // E
		Friction:  &mu,  // D = mu * Fz
	}
}
