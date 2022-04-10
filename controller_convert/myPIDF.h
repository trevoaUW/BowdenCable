//---Trevor Aquiningoc Position Controller
//---07-Apr-2022 16:49:04
    char        headerTime[] = "07-Apr-2022 16:49:04";
    int         PIDF_ns = 3;              // number of sections
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    static	struct	biquad PIDF[]={   // define the array of floating point biquads
        {2.112832e-03, 2.112840e-03, 0.000000e+00, 1.000000e+00, 9.135467e-01, 0.000000e+00, 0, 0, 0, 0, 0},
        {1.000000e+00, -1.993167e+00, 9.931715e-01, 1.000000e+00, -1.632606e+00, 6.734501e-01, 0, 0, 0, 0, 0},
        {1.000000e+00, 1.999996e+00, 9.999963e-01, 1.000000e+00, -1.998529e+00, 9.985303e-01, 0, 0, 0, 0, 0}
        };
