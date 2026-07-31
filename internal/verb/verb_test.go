package verb

import (
	"testing"

	"go.viam.com/test"
)

func TestSingle(t *testing.T) {
	tests := []struct {
		name    string
		cmd     map[string]interface{}
		want    string
		wantErr string
	}{
		{name: "single verb", cmd: map[string]interface{}{"detect": nil}, want: "detect", wantErr: ""},
		{name: "empty", cmd: map[string]interface{}{}, want: "", wantErr: "expected exactly one verb in DoCommand, got 0"},
		{name: "multiple verbs", cmd: map[string]interface{}{"detect": nil, "hint": true}, want: "", wantErr: "expected exactly one verb in DoCommand, got 2"},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got, err := Single(tt.cmd)
			if tt.wantErr == "" {
				test.That(t, err, test.ShouldBeNil)
				test.That(t, got, test.ShouldEqual, tt.want)
				return
			}
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldEqual, tt.wantErr)
		})
	}
}
