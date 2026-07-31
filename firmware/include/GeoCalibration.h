#pragma once

// Ported from geo_matrix.json (host tool: interactive_calibrate.py, 'g' key).
// Not auto-regenerated on-device — recalibrating means re-running 'g' on the
// host against fresh cal_*.log captures, then re-pasting these 6 rows here
// and reflashing. Kept separate from Calibration.h (DECOUPLING_M) since
// calibrate.py fully overwrites that file and knows nothing about this one.
//
// Ridge-regularized (toward identity) 6x6 decoupling matrix for the
// geometric model's own outputs — maps [px,py,dz,rx,ry,rz] (raw position in
// cm, raw orientation in degrees, from MotionController's plane-fit) to the
// same 6 values with cross-axis crosstalk suppressed. See
// interactive_calibrate.py's fit_geo_matrix() for why this needs
// regularization: a plain least-squares fit on 6 highly-correlated inputs
// badly overfits (coefficients up to 8-10x on the wrong axis).
// Rows: TX, TY, TZ, RX, RY, RZ  |  Cols: px, py, dz, rx, ry, rz

namespace Config {

const float GEO_M[6][6] = {
    {   0.946529f,    0.016210f,    0.021911f,   -0.003046f,    0.019347f,   -0.000312f },  // TX
    {   0.009190f,    0.967549f,    0.002842f,   -0.017103f,   -0.004471f,    0.014406f },  // TY
    {  -0.033139f,    0.003008f,    0.973066f,   -0.003042f,    0.014150f,    0.005003f },  // TZ
    {  -0.152348f,    0.195485f,    0.810437f,    0.656295f,   -0.203114f,   -0.274533f },  // RX
    {  -0.565729f,   -0.285683f,    0.711690f,   -0.380059f,    0.894043f,    0.023185f },  // RY
    {  -0.013049f,    0.389288f,    0.170590f,    0.152460f,   -0.027215f,    0.850735f }   // RZ
};

}  // namespace Config
