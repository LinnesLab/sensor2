#include "si115x_functions.h"

int16_t si115x_init( HANDLE si115x_handle )
{
    int16_t    retval;            

    retval  = Si115xReset( si115x_handle ); 
    delay_10ms(); 
    retval += Si115xParamSet( si115x_handle, PARAM_CH_LIST, 0x01); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG0, 0x78); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS0, 0x09); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST0, 0x40); 
    retval += Si115xWriteToRegister( si115x_handle, REG_IRQ_ENABLE, 0x01); 
    return retval;
}

//
// To start forced measurements         
//     Si115xForce( si115x_handle)      
//

typedef struct           
{                        
    uint8_t     irq_status;
    int32_t     ch0;
} SI115X_SAMPLES;                                 
                                                  
void si115x_handler(HANDLE si115x_handle,         
                    SI115X_SAMPLES *samples)      
{                                                 
    uint8_t buffer[4];                           
    Si115xBlockRead( si115x_handle,               
                      REG_IRQ_STATUS,             
                      4,                         
                      buffer);                    
    samples->irq_status = buffer[0];              
    samples->ch0  = buffer[1] << 16;
    samples->ch0 |= buffer[2] <<  8;
    samples->ch0 |= buffer[3];
    if( samples->ch0 & 0x800000 )   
        samples->ch0 |= 0xFF000000; 
}
#define X_ORDER_MASK 0x0070
#define Y_ORDER_MASK 0x0007
#define SIGN_MASK    0x0080
#define get_x_order(m)   ( (m & X_ORDER_MASK) >> 4 )
#define get_y_order(m)   ( (m & Y_ORDER_MASK)      )
#define get_sign(m)      ( (m & SIGN_MASK   ) >> 7 )

typedef struct {
  int16_t     info;
  uint16_t    mag;
} COEFF;

int32_t poly_inner( int32_t input,
               int8_t  fraction,
               uint16_t mag,
               int8_t  shift)
{    
  if (shift < 0)
  {
    return ( ( input << fraction ) / mag ) >> -shift ;
  }
  else
  {
    return ( ( input << fraction ) / mag ) << shift  ;
  }    
}

int32_t eval_poly( int32_t x, 
               int32_t y, 
               uint8_t input_fraction, 
               uint8_t output_fraction,
               uint8_t num_coeff, 
               COEFF  *kp
             )
{
  uint8_t  info, x_order, y_order, counter;
  int8_t   sign, shift;
  uint16_t mag;
  int32_t  output=0, x1, x2, y1, y2;

  for(counter=0; counter < num_coeff; counter++) 
  {
    info    = kp->info;
    x_order = get_x_order(info);
    y_order = get_y_order(info);

    shift   = ((uint16_t)kp->info&0xff00)>>8;
    shift  ^= 0x00ff;
    shift  += 1;
    shift   = -shift;    
    
    mag     = kp->mag;
    
    if( get_sign(info) ) sign = -1;
    else                 sign = 1;

    if( (x_order==0) && (y_order==0) )
    {
      output += sign * mag << output_fraction;
    }
    else
    {
      if( x_order > 0 )
      {
        x1 = poly_inner( x, input_fraction, mag, shift);
        if ( x_order > 1 )
        {
          x2 = poly_inner( x, input_fraction, mag, shift);
        }
        else
          x2 = 1;
      }
      else { x1 = 1; x2 = 1; }

      if( y_order > 0 )
      {
        y1 = poly_inner( y, input_fraction, mag, shift);
        if ( y_order > 1 )
        {
          y2 = poly_inner( y, input_fraction, mag, shift);
        }
        else
          y2 = 1;
      }
      else
      { y1 = 1; y2 = 1; }

      output += sign * x1 * x2 * y1 * y2;
    }
    kp++;
  }
  if( output < 0 ) output = -output;
  return output;
}

//
// Initialize UV coefficients
//
COEFF uk[2] = { {1537, 27440}, {2, 59952} };

#define UV_INPUT_FRACTION       15
#define UV_OUTPUT_FRACTION      12
#define UV_NUMCOEFF             2

//
// This is the main entry point for computing uv. The value returned by 
// get_uv is scaled by UV_OUTPUT_FRACTION
// 
// In order to get uvi as an integer, do this:
//
//   uvi = get_uv(uv, uk) / ( 1 << UV_OUTPUT_FRACTION )  
//
int32_t get_uv ( int32_t uv, 
                 COEFF *uk)
{
    int32_t uvi;

    uvi = eval_poly( 0, 
                     uv, 
                     UV_INPUT_FRACTION, 
                     UV_OUTPUT_FRACTION,
                     UV_NUMCOEFF,
                     uk );
    return uvi;
}

//
// General steps:
//
// 1. Initialize the Si115x
//
// 2. Initiate a conversion by using si115x_force()
//
// 3. The interrupt causes the interrupt handler to fill the
//    SI115X_SAMPLES structure
//
// 4. The example_calling_routine picks up data from the
//    SI115X_SAMPLES structure and calls the get_uv()
//    routine to compute the uvi
//
float uv_call_example( HANDLE si115x_handle, 
                        SI115X_SAMPLES *samples )
{
  float uvi;

  //
  // Example conversion to human-readable uvi values
  //
  uvi = (float) get_uv( samples->ch0, uk);
  uvi = uvi / ( 1 << UV_OUTPUT_FRACTION );

  return uvi;
}