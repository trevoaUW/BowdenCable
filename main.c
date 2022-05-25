/* Capstones  */

/* includes */
#include "stdio.h"
#include "MyRio.h"
#include "me477.h"
#include <string.h>
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

//change for controller
#include "myPIDF.h"
//acceleration controller
#include "myAcc.h"
//double derivative filter
#include "myDDeriv.h"

typedef struct {
NiFpga_IrqContext irqContext; // context
table *a_table; // table
seg *profile; // profile
int nseg; // no. of segs
NiFpga_Bool irqThreadRdy; // ready flag
} ThreadResource;


/* prototypes */
double cascade( double xin, // input
struct biquad *fa, // biquad array
int ns, // no. segments
double ymin, // min output
double ymax ); // max output
double pos(void);
void wait();
void *TimerIRQThread(void* resource);
int Sramps( seg *segs, // segments array
	int nseg, // number of segments
	int *iseg, // current segment index
	int *itime, // current time index
	double T, // sample period
	double *xa); // next reference posiiton

/* definitions */
#define SATURATE(x,lo,hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))
#define IMAX 50000 // max points

/* global variables*/
double KVI = 0.41;
double KT = 0.11;
double PI  = 3.14159265358979323846;
double J = 0.00034;

//already defined biquad,timeout,PIDF_ns
//encoder global variables
NiFpga_Session myrio_session;
MyRio_Encoder encC0;

// Buffers for Acceleration Control On
static double curpos_on[IMAX]; // currenpos buffer
static double *bpcurpos_on = curpos_on; // buffer pointer
static double refpos_on[IMAX]; // ref pos buffer
static double *bprefpos_on = refpos_on; // buffer pointer
static double torque_on[IMAX]; // torque buffer
static double *bptor_on = torque_on; // buffer pointer
static double curaccel_on[IMAX]; // current accel buffer
static double *bpcuraccel_on = curaccel_on; // buffer current accel pointer
static double refaccel_on[IMAX]; // ref accel buffer
static double *bprefaccel_on = refaccel_on; // buffer ref accel pointer

// Buffers for PID Only
static double curpos_off[IMAX]; // currenpos buffer
static double *bpcurpos_off = curpos_off; // buffer pointer
static double refpos_off[IMAX]; // ref pos buffer
static double *bprefpos_off = refpos_off; // buffer pointer
static double torque_off[IMAX]; // torque buffer
static double *bptor_off = torque_off; // buffer pointer
static double curaccel_off[IMAX]; // current accel buffer
static double *bpcuraccel_off = curaccel_off; // buffer current accel pointer
static double refaccel_off[IMAX]; // ref accel buffer
static double *bprefaccel_off = refaccel_off; // buffer ref accel pointer

// Velocity Collection Script if needed
//static double curvel[IMAX]; // current vel buffer
//static double *bpcurvel = curvel; // buffer current vel pointer
//static double refvel[IMAX]; // ref vel buffer
//static double *bprefvel = refvel; // buffer ref vel pointer

// Sramps and pos global vars
int itime = -1;
int iseg = -1;
int pos_int = 0;


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
    double dwell = 5; // s
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
	{"Ka:", 1, 5.0},
	{"Ke:", 0, 1.0},
	{"OnOff:", 1, 1.0},  // On: 1, Off: 0
    {"VDAout: (mV)", 0, 0.0 }
    };

    irqTimer0.timerWrite = IRQTIMERWRITE;
    irqTimer0.timerSet = IRQTIMERSETTIME;
    irqThread0.a_table = my_table;
    irqThread0.profile = mySegs;
    irqThread0.nseg =  nseg;

    //register DIO IRQ and the status terminate if not successful
    status = Irq_RegisterTimerIrq( &irqTimer0,
        						   &irqThread0.irqContext,
        						   BTI*1000000);

	// Set the indicator to allow the new thread.
	irqThread0.irqThreadRdy = NiFpga_True;


	// II. Create new thread to catch the IRQ.
	status = pthread_create(&thread,
							NULL,
							TimerIRQThread,
							&irqThread0);
	// calls the table editor
	ctable2(Table_Title, my_table, 8);

	// call signal interrupt to stop and wait to terminate
	irqThread0.irqThreadRdy = NiFpga_False;
	status = pthread_join(thread, NULL);

	//unregister DIO IRQ and terminate if not successful
		status = Irq_UnregisterTimerIrq(&irqTimer0,
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
	double *Ka = &((threadResource->a_table+4)->value);
	double *Ke = &((threadResource->a_table+5)->value);
	double *OnOff = &((threadResource->a_table+6)->value);
	double *VDAmV = &((threadResource->a_table+7)->value);


	seg *mySegs = threadResource->profile;
	int nseg = threadResource->nseg;
	double actualPosition;
	double actualAccel;
	double error;
	double torqueOut;
	double accelError;
	double OutputAccel;
	double AccelTorqueOutput;
	double vout;
	double denom_DD;
	double denom_ACC;
	int onFirst;
	int offFirst;
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
    		Irq_Wait(threadResource->irqContext,
    				TIMERIRQNO,
    				&irqAssert,
    				(NiFpga_Bool*) &(threadResource->irqThreadRdy));

    		// schedule the next IRQ
    		NiFpga_WriteU32( myrio_session,
    						IRQTIMERWRITE,
    						BTI*1000000);

    		NiFpga_WriteBool( myrio_session,
    						IRQTIMERSETTIME,
    						NiFpga_True);

    		//printf("Here 1");

    		//T =  BTILength / 1000.0;

    		//double refPosition;
    		// check if the IRQ  was asserted
    		if(irqAssert) {

    			if (*OnOff == 1.0){
    				if (onFirst == 0){
    					// Reset position
    					Aio_Write(&CO0,  0.0);
    					*pref = 0;
    					wait(3000);
    					*pact = 0;
    					// Reset Sramps
    					iseg = -1;
    					itime = -1;
    					pos_int = 0;
    					// Reset Buffer Pointers
    					bpcurpos_on = curpos_on; // buffer pointer
    					bprefpos_on = refpos_on; // buffer pointer
    					bptor_on = torque_on; // buffer pointer
    					bpcuraccel_on = curaccel_on; // buffer current accel pointer
    					bprefaccel_on = refaccel_on; // buffer ref accel pointer
    					// Reset Status Flags
    					onFirst = 1;
    					offFirst = 0;
    				}
    				ramps = Sramps(mySegs, nseg, &iseg, &itime, BTI, pref); //rev
    				*Ke = 1;
    				//PIDF Controller
					actualPosition = pos() / 2000; //rev
					*pact = actualPosition;
					error = (*pref - *pact)*2*PI; //rad
					torqueOut = cascade(error, PIDF, PIDF_ns, -0.451, 0.451); //get PID output

					// Acceleration Controller
					*aref = torqueOut / J;  //rad/s^2
					// Double Derivative
					denom_DD = (4*pow(tau_DD,2))+(4*BTI*tau_DD)+pow(BTI, 2);
					DDERIV->b0 = 4*(*Ke)/ denom_DD;
					DDERIV->b1 = -8*(*Ke)/ denom_DD;
					DDERIV->b2 = DDERIV->b0;
					DDERIV->a1 = (-8*pow(tau_DD, 2) + 2*pow(BTI,2))/denom_DD;
					DDERIV->a2 = ((4*pow(tau_DD,2))-(4*BTI*tau_DD)+pow(BTI, 2))/denom_DD;
					actualAccel = cascade((*pact)*2*PI, DDERIV, DDERIV_ns,-1346.5, 1326.5); //convert rev to rad
					*aact = actualAccel; //rad/s^2
					accelError = *aref - *aact;//rad/s^2 output of accel. controller
					denom_ACC = (2*tau_ACC)+BTI;
					ACC -> b0 = (*Ka)*BTI/denom_ACC;
					ACC -> b1 = ACC -> b0;
					ACC -> a1 = (-(2*tau_ACC)+BTI)/denom_ACC;
					OutputAccel = cascade(accelError, ACC, ACC_ns,-1326.5, 1326.5);
					AccelTorqueOutput = OutputAccel * J;
					vout = AccelTorqueOutput / KT / KVI;
					Aio_Write(&CO0,  vout);

					// Write MATLAB Data
					//measured position
					if (bpcurpos_on < curpos_on + IMAX){
						*bpcurpos_on++ = actualPosition*2*PI;
					}
					//reference position
					if (bprefpos_on < refpos_on + IMAX){
						*bprefpos_on++ = (*pref)*(2*PI);
					}
					//add torque
					if (bptor_on < torque_on + IMAX){
						*bptor_on++ =  KVI*KT*vout;
					}
					//reference acceleration
					if (bprefaccel_on < refaccel_on + IMAX){
						*bprefaccel_on++ = (*aref)*(2*PI);
					}
					// measured current accel
					if (bpcuraccel_on < curaccel_on + IMAX){
							*bpcuraccel_on++ = actualAccel*(2*PI);
					 }

    			}
    			else if (*OnOff == 0.0){
    				if (offFirst == 0){
    					// Reset Position
    					Aio_Write(&CO0,  0.0);
    					*pref = 0;
    					wait(3000);
    					*pact = 0;
    					// Reset Sramps
						iseg = -1;
						itime = -1;
						pos_int = 0;
						// Reset Buffer Pointers
						bpcurpos_off = curpos_off; // buffer pointer
						bprefpos_off = refpos_off; // buffer pointer
						bptor_off = torque_off; // buffer pointer
						bpcuraccel_off = curaccel_off; // buffer current accel pointer
						bprefaccel_off = refaccel_off; // buffer ref accel pointer
						// Update status flags
						onFirst = 0;
						offFirst = 1;
					}
    				ramps = Sramps(mySegs, nseg, &iseg, &itime, BTI, pref); //rev
    				*Ke = 0;
    				actualPosition = pos() / 2000; //rev
					*pact = actualPosition;
					error = (*pref - *pact)*2*PI; //rad
					torqueOut = cascade(error, PIDF, PIDF_ns, -0.451, 0.451); //get PID output
					vout = torqueOut / KT / KVI;
					Aio_Write(&CO0,  vout);

					// Write MATLAB Data
					//measured position
					if (bpcurpos_off < curpos_off + IMAX){
						*bpcurpos_off++ = actualPosition*2*PI;
					}
					//reference position
					if (bprefpos_off < refpos_off + IMAX){
						*bprefpos_off++ = (*pref)*(2*PI);
					}
					//add torque
					if (bptor_off < torque_off + IMAX){
						*bptor_off++ =  KVI*KT*vout;
					}
					//reference acceleration
					if (bprefaccel_off < refaccel_off + IMAX){
						*bprefaccel_off++ = (*aref)*(2*PI);
					}
					// measured current accel
					if (bpcuraccel_off < curaccel_off + IMAX){
							*bpcuraccel_off++ = actualAccel*(2*PI);
					}
    			}
    			*VDAmV =   vout * 1000.0;
    		    Irq_Acknowledge(irqAssert);
    		}
    	}

		// convert data to a MATLAB file for further analysis
		MATFILE * mf;
		int err;

		mf = openmatfile("CapstoneDataNew.mat", &err);
		if(!mf){
				printf("Can’t open mat file %d\n", err);
			}
		matfile_addstring(mf, "myName", "Group 2");
		matfile_addmatrix(mf, "Refpos_on", refpos_on, IMAX, 1, 0);
		matfile_addmatrix(mf, "Curpos_on", curpos_on, IMAX, 1, 0);
		matfile_addmatrix(mf, "Refaccel_on", refaccel_on, IMAX, 1, 0);
		matfile_addmatrix(mf, "Curaccel_on", curaccel_on, IMAX, 1, 0);
		matfile_addmatrix(mf, "Torque_on", torque_on, IMAX, 1, 0);
		matfile_addmatrix(mf, "Refpos_off", refpos_off, IMAX, 1, 0);
		matfile_addmatrix(mf, "Curpos_off", curpos_off, IMAX, 1, 0);
		matfile_addmatrix(mf, "Refaccel_off", refaccel_off, IMAX, 1, 0);
		matfile_addmatrix(mf, "Curaccel_off", curaccel_off, IMAX, 1, 0);
		matfile_addmatrix(mf, "Torque_off", torque_off, IMAX, 1, 0);
		matfile_addmatrix(mf, "PIDF", (double *) PIDF, 6, 1, 0);
		matfile_addmatrix(mf, "AccelController", (double *) ACC, 6, 1, 0);
		matfile_addmatrix(mf, "DoubleDervivate", (double *) DDERIV, 6, 1, 0);
		matfile_addmatrix(mf, "BTILength", &BTI, 1, 1, 0);
		matfile_addmatrix(mf, "Ka", Ka, 1, 1, 0);
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
	static int32_t curCount;
	static int32_t prevCount;

	// reading the current encoder counter
	if (pos_int == 0) {
		curCount = Encoder_Counter(&encC0);
		prevCount = curCount;
		pos_int = 1;
	} else {
		curCount = Encoder_Counter(&encC0);
	}
	// taking a difference between current and previous encoder counter
	double delta = (double) (curCount -prevCount);
	return delta;
}

/*--------------------------------------------------------------
 method: wait
 purpose: wait a given amount of milliseconds
 parameters: takes in number of ms intended to pass
 returns: none
-------------------------------------------------------------- */
void wait(int cycles) {
    uint32_t i;
    i = (417000/5)*cycles;
    while(i>0){
        i--;
    }
    return;
}
