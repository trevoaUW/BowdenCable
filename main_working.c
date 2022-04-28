/* Capstones  */

/* includes */
#include "stdio.h"
#include "MyRio.h"
#include "me477.h"
#include <string.h>
//#include "emulate.h"
#include "ctable2.h"
#include "TimerIRQ.h"
#include <pthread.h>
#include "matlabfiles.h"
#include "IRQConfigure.h"
#include "Encoder.h"
#include "math.h"

typedef struct {double xfa; // position
double v; // velocity limit
double a; // acceleration limit
double d; // dwell time (s)
} seg;

struct biquad {
double b0; double b1; double b2; // numerator
double a0; double a1; double a2; // denominator
double x0; double x1; double x2; // input
double y1; double y2; }; // output

/*

*/
//change for controller
#include "myPIDF.h"
//acceleration controller (change by its saved file)
#include "myAcc.h"
//converts position into acceleration for feeding back
#include "myDDeriv.h"


typedef struct {
NiFpga_IrqContext irqContext; // context
table *a_table; // table
seg *profile; // profile
int nseg; // no. of segs
NiFpga_Bool irqThreadRdy; // ready flag
} ThreadResource;


/* prototypes */
//double	double_in(char *prompt);
double cascade( double xin, // input
struct biquad *fa, // biquad array
int ns, // no. segments
double ymin, // min output
double ymax ); // max output
double pos(void);
void *TimerIRQThread(void* resource);
int Sramps( seg *segs, // segments array
	int nseg, // number of segments
	int *iseg, // current segment index
	int *itime, // current time index
	double T, // sample period
	double *xa); // next reference posiiton

/* definitions */
#define SATURATE(x,lo,hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))
#define IMAX 4000 // max points

/* global variables*/
double KVI = 0.41;
double KT = 0.11;
double PI  = 3.14159265358979323846;
double J = 0.00034;

//already defined biquad,timeout,PIDF_ns
//encoder global variables
NiFpga_Session myrio_session;
MyRio_Encoder encC0;

double BTILength = 5; //ms
static double curpos[IMAX]; // current pos buffer
static double *bpcurpos = curpos; // buffer pointer
static double refpos[IMAX]; // ref pos buffer
static double *bprefpos = refpos; // buffer pointer
static double torque[IMAX]; // torque buffer
static double *bptor = torque; // buffer pointer

// if we have more time, build another filter for 1 derivative
//static double curvel[IMAX]; // current vel buffer
//static double *bpcurvel = curvel; // buffer current vel pointer
//static double refvel[IMAX]; // ref vel buffer
//static double *bprefvel = refvel; // buffer ref vel pointer

static double curaccel[IMAX]; // current accel buffer
static double *bpcuraccel = curaccel; // buffer current accel pointer
static double refaccel[IMAX]; // ref accel buffer
static double *bprefaccel = refaccel; // buffer ref accel pointer


int itime = -1;
int iseg = -1;


/*--------------------------------------------------------------
Function main
Purpose:
1. initialized the table editor for the user to view on LCD screen and the path pofile variable
2. setup the connection to the myRio and the timer Interrupt Service Routine (ISR)
3. The time thread is registered and create a new thread to catch
the interrupts
3. calls the table editor method and initialize all its variables
4. After the user press the backspace, the table editor exit which stop and wait
for the interrupt to terminate
5. Lastly the connection of the MyRio is terminated

Parameters: argc - total number of arguments
argv is the array of strings of the commands passed by the arguments
Returns: int - determine if the status of the MyRio is success or not
--------------------------------------------------------------*/
int main(int argc, char **argv)
{
	NiFpga_Status status1;

    status1 = MyRio_Open();		    /*Open the myRIO NiFpga Session.*/
    if (MyRio_IsNotSuccess(status1)) return status1;

    int32_t status;
    MyRio_IrqTimer irqTimer0;
    ThreadResource irqThread0;
    pthread_t thread;

    //my code here
    double vmax = 50.; // revs/s
    double amax = 20.; // revs/s^2
    double dwell = 3; // s
    seg mySegs[8] = { // revs
    {10.125, vmax, amax, dwell},
    {20.250, vmax, amax, dwell},
    {30.375, vmax, amax, dwell},
    {40.500, vmax, amax, dwell},
    {30.625, vmax, amax, dwell},
    {20.750, vmax, amax, dwell},
    {10.875, vmax, amax, dwell},
    { 0.000, vmax, amax, dwell}
    };
    int nseg = 8;


    //initialize the table editor variables
    char *Table_Title ="Control Table";
    table my_table[] = {
    {"P ref: (revs)", 0, 0.0 },
    {"P act: (revs)", 0, 0.0 },
    {"A ref:", 0, 0.0},
    {"A act:", 0, 0.0},
    {"VDAout: (mV)", 0, 0.0 },
    };

    irqTimer0.timerWrite = IRQTIMERWRITE;
    irqTimer0.timerSet = IRQTIMERSETTIME;
    irqThread0.a_table = my_table;
    irqThread0.profile = mySegs;
    irqThread0.nseg =  nseg;

    //register DIO IRQ and the status terminate if not successful
    status = Irq_RegisterTimerIrq( &irqTimer0,
        								&irqThread0.irqContext,
        								timeoutValue);

	// Set the indicator to allow the new thread.
	irqThread0.irqThreadRdy = NiFpga_True;


	// II. Create new thread to catch the IRQ.
	status = pthread_create( &thread,
							NULL,
							TimerIRQThread,
							&irqThread0);
	// calls the table editor
	ctable2(Table_Title, my_table, 5);

	// call signal interrupt to stop and wait to terminate
	irqThread0.irqThreadRdy = NiFpga_False;
	status = pthread_join(thread, NULL);

	//unregister DIO IRQ and terminate if not successful
		status = Irq_UnregisterTimerIrq( &irqTimer0,
										irqThread0.irqContext);

	status1 = MyRio_Close();	 /*Close the myRIO NiFpga Session. */
	return status1;
}

/*-----------------------------------
Function TimerIRQThread
Purpose:
1. a new thread resource is created. The analog output are initialized by
setting the analog output to be zero volts.
The encoder is setup to interact with the FPGA interface.
2. The new threads waits to be asserted. Afterwards, the timer write register is written and
write true for the Timer set time register.
3. The computed position reference is calculated in the Sramp function
4. The actual position of the motor is measured with calling the pos function.
5. An difference error is computed between the reference and actual position.
The cascade method is called
with this error as its input, the PIDF filter from MATLAB,
and the difference equation to compute its output.
6. The system digital output is written to the Digital to Analog converted
The reference and actual positions, torque, system output values, and BTI value
are recorded to a MATLAB file for further analysis
7. The new thread resource is terminated after the main thread
stop calling it

Parameters: void* resource - a void pointer to point to a new
thread Resource

Returns: a void pointer which is NULL
*-----------------------------------*/
void *TimerIRQThread(void* resource) {
	//cast the resource pointer to a thread resource
	ThreadResource* threadResource = (ThreadResource*) resource;
	extern NiFpga_Session myrio_session;

	double *pref = &((threadResource->a_table+0)->value);
	double *pact = &((threadResource->a_table+1)->value);
	double *aref = &((threadResource->a_table+2)->value);
	double *aact = &((threadResource->a_table+3)->value);
	double *VDAmV = &((threadResource->a_table+4)->value);


	seg *mySegs = threadResource->profile;
	int nseg = threadResource->nseg;
	double actualPosition;
	double actualAccel;
	double T;
	double error;
	double torqueOut;
	double accelError;
	double OutputAccel;
	double AccelTorqueOutput;
	double vout;
	int ramps;

	MyRio_Aio CO0;
	MyRio_Aio CI0;
    AIO_initialize(&CI0, &CO0);
    // motor voltage to zero
    Aio_Write(&CO0, 0);

    // initializing the quadrature encoder counter
    	// and setting the counter to zero
        EncoderC_initialize(myrio_session, &encC0);


    	uint32_t irqAssert = 0;
    	// check if main thread does not signal to stop this thread
    	while (threadResource->irqThreadRdy == NiFpga_True) {
    		// wait for the interrupt
    		Irq_Wait( threadResource->irqContext,
    				TIMERIRQNO,
    				&irqAssert,
    				(NiFpga_Bool*) &(threadResource->irqThreadRdy));

    		// schedule the next IRQ
    		NiFpga_WriteU32( myrio_session,
    						IRQTIMERWRITE,
    						timeoutValue);

    		NiFpga_WriteBool( myrio_session,
    						IRQTIMERSETTIME,
    						NiFpga_True);

    		 T =  BTILength / 1000.0;

    		//double refPosition;
    		// check if the IRQ  was asserted
    		if(irqAssert & (1 << TIMERIRQNO)) {
    			//*pref = *pref*2*PI;
    			ramps = Sramps(mySegs, nseg, &iseg, &itime, T, pref); //rev
    			actualPosition = pos() / 2000; //rev
    			*pact = actualPosition;
    			error = (*pref - *pact)*2*PI; //rad THIS WAS CAUSING ISSUE!!! Pref and Pact

				//grab aact from physical system, assign to table
				//grab aref from table

				torqueOut = cascade(error, PIDF, PIDF_ns,-1, 1); //get PID output
				*aref = torqueOut / J;  //rad/s^2
				actualAccel = cascade((*pact)*2*PI, DDERIV, DDERIV_ns,-900,900); //convert rev to rad
				*aact = actualAccel; //rad/s^2
				accelError = *aref - *aact;//rad/s^2 output of accel. controller
				OutputAccel = cascade(accelError, ACC, ACC_ns,-900,900);
				AccelTorqueOutput = OutputAccel * J;

				vout = AccelTorqueOutput / KT / KVI;
				Aio_Write(&CO0,  vout);

    		    //measured position
    		    if (bpcurpos < curpos + IMAX){

    		    	*bpcurpos++ = actualPosition*2*PI;
    		 	}

    		    //reference position
    		    if (bprefpos < refpos + IMAX){

    		    	*bprefpos++ = (*pref)*(2*PI);
    		 	}

    		    //add torque
    		    if (bptor < torque + IMAX){
    		 		*bptor++ =  KVI*KT*vout;
    		 	}

                //reference acceleration
       		    if (bprefaccel < refaccel + IMAX){

       		    	*bprefaccel++ = (*aref)*(2*PI);
       		 	}

                // measured current accel
        		if (bpcuraccel < curaccel + IMAX){

        		    	*bpcuraccel++ = actualAccel*(2*PI);
        		 }

    			*VDAmV =   vout * 1000.0;

    		   Irq_Acknowledge(irqAssert);
    		}
    	}

		// convert data to a MATLAB file for further analysis
		MATFILE * mf;
			int err;

		   	mf = openmatfile("CapstoneData.mat", &err);
			if(!mf){
				printf("Can’t open mat file %d\n", err);
			}
			matfile_addstring(mf, "myName", "Group 2");
			matfile_addmatrix(mf, "Refpos", refpos, IMAX, 1, 0);
			matfile_addmatrix(mf, "Curpos", curpos, IMAX, 1, 0);
			matfile_addmatrix(mf, "Refaccel", refaccel, IMAX, 1, 0);
			matfile_addmatrix(mf, "Curaccel", curaccel, IMAX, 1, 0);
			matfile_addmatrix(mf, "Torque", torque, IMAX, 1, 0);
			matfile_addmatrix(mf, "PIDF", (double *) PIDF, 6, 1, 0);
			matfile_addmatrix(mf, "AccelController", (double *) ACC, 6, 1, 0);
			matfile_addmatrix(mf, "DoubleDervivate", (double *) DDERIV, 6, 1, 0);
			matfile_addmatrix(mf, "BTILength", &T, 1, 1, 0);
			matfile_close(mf);


			pthread_exit(NULL);
			return NULL;
}


/*-----------------------------------
Function cascade
Purpose:Using the Biquad Cascade technique,calculate the system output based on the current system input
and the string of biquad sections
Parameters: double xin - current ADC input
			struct biquad *fa - biquad array with coefficients and history of inputs and outputs
			int ns - number of biquad segments
			double ymin - minimum saturate limit
			double ymax  - maximum saturate limit

Returns: double - current value of the system output
*-----------------------------------*/
double cascade( double xin, struct biquad *fa,
				int ns, // no. segments
				double ymin, // min output
				double ymax ){ // max output

	double y0;
	struct biquad *f;
	f = fa;
	int i;
	y0 = xin;
	for (i = 0; i < ns; i++) {
		f->x0 = y0;
		y0 = (f->b0*f->x0 + f->b1*f->x1+ f->b2*f->x2 - (f->a1*f->y1) - (f->a2*f->y2)) /f->a0;
		f->x2 = f->x1;
		f->x1 = f->x0;
		f->y2 = f->y1;
		f->y1 = y0;
		f++;
	}
	y0 = SATURATE(y0, ymin, ymax);
	return y0;
}

/*--------------------------------------------------------------
Function pos
Purpose: read the current count from the encoder counter, take the difference of the previous count and current count
(the current and previous count are the same when this function is first called), and set the previous count to the
current count
Parameters: void
Returns: double - difference between the current and previous positions Th
--------------------------------------------------------------*/
double pos(void) {
	static int n = 1;
	static int32_t curCount;
	static int32_t prevCount;

	// reading the current encoder counter
	if (n == 1) {
		curCount = Encoder_Counter(&encC0);
		prevCount = curCount;
		n = 0;
	} else {
		curCount = Encoder_Counter(&encC0);
	}
	// taking a difference between current and previous encoder counter
	double delta = (double) (curCount -prevCount);
	return delta;
}




