//---Trevor Aquiningoc Position Controller
//---21-Apr-2022 15:18:17
    char        headerTime[] = "21-Apr-2022 15:18:17";
    int         PIDF_ns = 1;              // number of sections
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    static	struct	biquad PIDF[]={   // define the array of floating point biquads
        {2.003843e-01, -3.993994e-01, 1.990159e-01, 1.000000e+00, -1.815718e+00, 8.157181e-01, 0, 0, 0, 0, 0}
        };
