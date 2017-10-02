/* Motor controller sensors 
 *
 * Encoder - Mixed QEI / GPIO, High frequency
 *
 * Current - ADC, High frequency
 *
 * Vbus    - ADC, High frequency
 *
 *
 * Temp    - ADC, Low frequency
 * 
 * Vbat    - ADC, low frequency
 *
 */



/*

Consumers of Encoder data:
 - Control loop  OR  External control loop
 - Max velocity monitor
 - Status update

Consumers of Current data:
 - Control loop  OR  External control loop
 - Max current monitor
 - H-Bridge regen monitor
 - Status update

Consumers of Vbus data:
 - H-Bridge regen monitor
 - Status update

Consumers of Temp data:
 - Max temp monitor
 - Status update

Consumers of Vbat data:
 - Min Vbat monitor
 - Status update


*/


/* Sampling rates:

 * All sensors will be sampled at a fixed primary sample rate, P.

 * The primary sample rate will be used by the closed-loop controller 
   and safety functions.

 * A configurable decimating filter will be used to provide a secondary
   sample rate, S, where S = P/N, for N an integer in a pre-defined range.

 * The external control mechanism and automated status update mechanism
   will use the secondary sample rate. 

*/
