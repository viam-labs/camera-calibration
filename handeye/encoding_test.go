package handeye

import (
	"math"
	"testing"

	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"gonum.org/v1/gonum/num/quat"
)

// geodesicDeg returns the angle between two rotations (quaternions) in degrees.
// Uses |q1·q2| = cos(θ/2) to be sign-agnostic.
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

// TestSolveOrientationEncodingRoundTrip exercises the exact rvec→R4→OV encoding
// path used in solveResponse.toResult. A systematic encoding bug (e.g. in
// R3ToR4 or OrientationVectorDegrees) would show as a nonzero geodesic delta
// between the input rotation and the one recovered from the resulting OV.
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

			// Simulate what the Python solver emits: rvec = axis * angle (Rodrigues).
			rvec := spatialmath.QuatToR3AA(qIn)

			// Run the exact production pipeline from solveResponse.toResult.
			ovd := spatialmath.R3ToR4(rvec).OrientationVectorDegrees()

			// Decode OV back to a quaternion via spatialmath's own path.
			qOut := ovd.Quaternion()

			delta := geodesicDeg(qIn, qOut)
			t.Logf("in q=(%.4f, %.4f, %.4f, %.4f) → OV(OX=%.4f, OY=%.4f, OZ=%.4f, Θ=%.4f°) → geodesic Δ=%.6f°",
				qIn.Real, qIn.Imag, qIn.Jmag, qIn.Kmag, ovd.OX, ovd.OY, ovd.OZ, ovd.Theta, delta)
			test.That(t, delta, test.ShouldAlmostEqual, 0.0, 1e-6)
		})
	}
}

// TestNjsanding2SolveVsHandTunedOrientation is a focused diagnostic for the
// 2026-10-22 run on njsanding2. If encoding is lossless and the solve re-found
// the hand-tuned frame, the two OVs below should encode nearly the same rotation.
// A large geodesic delta here indicates either (a) a real orientation disagreement
// between solver and hand-tuned reference, or (b) a bug upstream (Python → Go
// handoff) that was not caught by the self-consistent round-trip above.
func TestNjsanding2SolveVsHandTunedOrientation(t *testing.T) {
	// Current config: sensing-camera frame on njsanding2 (hand-tuned to "perfect").
	handTuned := normalizeQuat(quat.Number{
		Real: 0.7043302478473606,
		Imag: -0.007432235254731113,
		Jmag: 0.024795400764921018,
		Kmag: 0.7094003467346918,
	})

	// Solve output from the 2026-10-22 run.
	solvedOV := &spatialmath.OrientationVectorDegrees{
		OX:    0.02389948760818241,
		OY:    0.049794624846892795,
		OZ:    0.9984734897975125,
		Theta: 26.539732991637624,
	}
	solvedOV.Normalize()
	qSolved := solvedOV.Quaternion()

	delta := geodesicDeg(handTuned, qSolved)
	t.Logf("hand-tuned vs solved geodesic Δ = %.4f°", delta)

	// If this prints >1°, orientations really differ — look at step 2 (dump
	// the Python-side rotation matrix pre-handoff). If it prints ~0°, the
	// earlier Python-side analysis used a different OV convention than
	// spatialmath, and solver orientation is actually correct.
	//
	// Not asserting equality here — this is a diagnostic log, not a gate.
}
