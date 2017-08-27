#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define MAXLOOPS 50

// Controller coefficients
#define c0	0.1
#define c1	0.1
#define c2	0.1
#define c3	0.1
#define c4	0.1

// Plant coefficients
#define MOTOR_CONSTANT		0.0398   // Nm/A
#define MOMENT_OF_INERTIA	0.001256 // kgm^2
#define ARMATURE_RESISTANCE	0.089    // ohms
#define ARMATURE_INDUCTANCE	0.00012  // henries
#define VISCOUS_FRICTION	0.0001    // ???

#define A (MOTOR_CONSTANT / (MOMENT_OF_INERTIA * ARMATURE_INDUCTANCE))
#define B ((VISCOUS_FRICTION / MOMENT_OF_INERTIA) +		\
	     (ARMATURE_RESISTANCE / ARMATURE_INDUCTANCE))
#define C (((VISCOUS_FRICTION * ARMATURE_RESISTANCE) +	\
	      (MOTOR_CONSTANT * MOTOR_CONSTANT)) /		\
	     (MOMENT_OF_INERTIA * ARMATURE_INDUCTANCE))
#define alpha1 (-2 -B)
#define alpha2 (1 + B + C)

#define p0	(-1 / alpha2)
#define p1	(-alpha1 / alpha2)
#define p2	(0.001 * A / alpha2)


#define PID_LOOP_FREQ   1000
#define PID_KP	        20
#define PID_KI	        0 // 10
#define PID_KD	        0 // 5
#define PID_DT		(1.0/(PID_LOOP_FREQ))



// The PID controller that is run on the MCU.
float pid_calculate(float target, float measured)
{
  static float previous_error;
  static float integral;
  float error, derivative, output;
  
  error = target - measured;

  integral += error * PID_DT;
  derivative = (error - previous_error) / PID_DT;
  output = PID_KP * error + PID_KI * integral + PID_KD * derivative;
  previous_error = error;

  return output;
}    

// The plant simulation that is run on the PC
float plant_calculate(float input)
{
  static float u1;
  static float u2;

  float u = p0*u1 + p1*u2 + p2*input;

  printf("%f %f %f %f %f\n", input, p0*u1, p1*u2, p2*input, u);
    
  u2 = u1;
  u1 = input;
  
  return u;
}


void print_output(float error_signal, float controller_output, float plant_output)
{
  printf("error signal:       %f\n", error_signal);
  printf("controller output:  %f\n", controller_output);
  printf("plant output:       %f\n", plant_output);
  putchar('\n');
  sleep(1);
}


void main(void)
{
  float target = 10.0;

  //  printf("%f %f %f\n", p0, p1, p2);
  //  exit(1);

  
  float plant_output = 0;
  float controller_output = 0;
  float error_signal = 0;

  int i;
  for (i = 0; i < MAXLOOPS; i++) {
    // One iteration of the controller/plant simulation
    error_signal = target - plant_output;
    controller_output = pid_calculate(target, plant_output);
    plant_output = plant_calculate(controller_output);

    // Results to be read by Octave
    //    printf("%f %f %f\n", error_signal, controller_output, plant_output);
  }
}

/* // The PID controller that is run on the MCU. */
/* float pid_calculate(float error) */
/* { */
/*   static float u1;	// u[k-1] */
/*   static float u2;	// u[k-2] */
/*   static float e1;	// e[k-1] */
/*   static float e2;	// e[k-2] */

/*   // Next controller output */
/*   float u = c0*u1 + c1*u2 + c2*error + c3*e1 + c4*e2; */

/*   // Update controller state */
/*   u2 = u1; */
/*   u1 = u; */
/*   e2 = e1; */
/*   e1 = error; */

/*   return u; */
/* } */
