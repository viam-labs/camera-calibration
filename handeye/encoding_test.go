package handeye

import (
	"math"
	"testing"

	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"gonum.org/v1/gonum/num/quat"
)

func geodesicDeg(q1, q2 quat.Number) float64 {
	dot := q1.Real*q2.Real + q1.Imag*q2.Imag + q1.Jmag*q2.Jmag + q1.Kmag*q2.Kmag
	if dot < 0 {
		dot = -dot
	}
	if dot > 1 {
		dot = 1
	}
	return 2 * math.Acos(dot) * 180 / math.Pi
}

func normalizeQuat(q quat.Number) quat.Number {
	n := math.Sqrt(q.Real*q.Real + q.Imag*q.Imag + q.Jmag*q.Jmag + q.Kmag*q.Kmag)
	return quat.Number{Real: q.Real / n, Imag: q.Imag / n, Jmag: q.Jmag / n, Kmag: q.Kmag / n}
}

// Pins the rvec→R4→OV path from solveResponse.toResult against orientation loss.
func TestSolveOrientationEncodingRoundTrip(t *testing.T) {
	tests := []struct {
		name string
		q    quat.Number // (w, x, y, z)
	}{
		{name: "identity", q: quat.Number{Real: 1}},
		{name: "90deg about X", q: quat.Number{Real: math.Cos(math.Pi / 4), Imag: math.Sin(math.Pi / 4)}},
		{name: "45deg about Y", q: quat.Number{Real: math.Cos(math.Pi / 8), Jmag: math.Sin(math.Pi / 8)}},
		{name: "120deg about diag", q: quat.Number{Real: 0.5, Imag: 0.5, Jmag: 0.5, Kmag: 0.5}},
		// njsanding2's current hand-tuned frame.
		{name: "njsanding2 hand-tuned", q: quat.Number{
			Real: 0.7043302478473606,
			Imag: -0.007432235254731113,
			Jmag: 0.024795400764921018,
			Kmag: 0.7094003467346918,
		}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			qIn := normalizeQuat(tt.q)
			rvec := spatialmath.QuatToR3AA(qIn)
			ovd := spatialmath.R3ToR4(rvec).OrientationVectorDegrees()
			qOut := ovd.Quaternion()

			delta := geodesicDeg(qIn, qOut)
			t.Logf("in q=(%.4f, %.4f, %.4f, %.4f) → OV(OX=%.4f, OY=%.4f, OZ=%.4f, Θ=%.4f°) → geodesic Δ=%.6f°",
				qIn.Real, qIn.Imag, qIn.Jmag, qIn.Kmag, ovd.OX, ovd.OY, ovd.OZ, ovd.Theta, delta)
			test.That(t, delta, test.ShouldAlmostEqual, 0.0, 1e-6)
		})
	}
}

