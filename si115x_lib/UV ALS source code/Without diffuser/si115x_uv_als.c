#include "si115x_functions.h"

int16_t si115x_init( HANDLE si115x_handle )
{
    int16_t    retval;            

    retval  = Si115xReset( si115x_handle ); 
    delay_10ms(); 
    retval += Si115xParamSet( si115x_handle, PARAM_CH_LIST, 0x0f); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG0, 0x78); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS0, 0x71); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST0, 0x40); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG1, 0x4d); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS1, 0xe1); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST1, 0x40); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG2, 0x41); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS2, 0xe1); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST2, 0x50); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG3, 0x4d); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS3, 0x87); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST3, 0x40); 
    retval += Si115xWriteToRegister( si115x_handle, REG_IRQ_ENABLE, 0x0f); 
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
    int32_t     ch1;
    int32_t     ch2;
    int32_t     ch3;
} SI115X_SAMPLES;                                 
                                                  
void si115x_handler(HANDLE si115x_handle,         
                    SI115X_SAMPLES *samples)      
{                                                 
    uint8_t buffer[13];                           
    Si115xBlockRead( si115x_handle,               
                      REG_IRQ_STATUS,             
                      13,                         
                      buffer);                    
    samples->irq_status = buffer[0];              
    samples->ch0  = buffer[1] << 16;
    samples->ch0 |= buffer[2] <<  8;
    samples->ch0 |= buffer[3];
    if( samples->ch0 & 0x800000 )   
        samples->ch0 |= 0xFF000000; 
    samples->ch1  = buffer[4] << 16;
    samples->ch1 |= buffer[5] <<  8;
    samples->ch1 |= buffer[6];
    if( samples->ch1 & 0x800000 )   
        samples->ch1 |= 0xFF000000; 
    samples->ch2  = buffer[7] << 16;
    samples->ch2 |= buffer[8] <<  8;
    samples->ch2 |= buffer[9];
    if( samples->ch2 & 0x800000 )   
        samples->ch2 |= 0xFF000000; 
    samples->ch3  = buffer[10] << 16;
    samples->ch3 |= buffer[11] <<  8;
    samples->ch3 |= buffer[12];
    if( samples->ch3 & 0x800000 )   
        samples->ch3 |= 0xFF000000; 
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
COEFF uk[2] = { {1281, 30902}, {-638, 46301} };

#define UV_INPUT_FRACTION       15
#define UV_OUTPUT_FRACTION      12
#define UV_NUMCOEFF             2

//
// This is the main entry point for computing uv. The value returned by 
// get_uv is scaled by UV_OUTPUT_FRACTION
// 
// In order to get lux as an integer, do this:
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
  // Example conversion to human-readable lux values
  //
  uvi = (float) get_uv( samples->ch0, uk);
  uvi = uvi / ( 1 << UV_OUTPUT_FRACTION );

  return uvi;
}

typedef struct {
  COEFF   coeff_high[4];
  COEFF   coeff_low[9];
} LUX_COEFF; 

//
// Initialize coefficients
//
LUX_COEFF lk ={ { {0, 209},      // coeff_high[0]
                  {1665, 93},      // coeff_high[1]
                  {2064, 65},      // coeff_high[2]
                  {-2671, 234} },    // coeff_high[3]
                { {0, 0},      // coeff_low[0]
                  {1921, 29053},      // coeff_low[1]
                  {-1022, 36363},      // coeff_low[2]
                  {2320, 20789},      // coeff_low[3]
                  {-367, 57909},      // coeff_low[4]
                  {-1774, 38240},      // coeff_low[5]
                  {-608, 46775},      // coeff_low[6]
                  {-1503, 51831},      // coeff_low[7]
                  {-1886, 58928} } };  // coeff_low[8]

#define ADC_THRESHOLD           16000
#define INPUT_FRACTION_HIGH     7
#define INPUT_FRACTION_LOW      15
#define LUX_OUTPUT_FRACTION     12
#define NUMCOEFF_LOW            9
#define NUMCOEFF_HIGH           4

//
// This is the main entry point for computing lux. The value returned by 
// get_lux is scaled by LUX_OUTPUT_FRACTION
// 
// In order to get lux as an integer, do this:
//
//   lux = get_lux(vis_high, vis_low, ir, &lk) / ( 1 << LUX_OUTPUT_FRACTION )  
//
int32_t get_lux( int32_t vis_high, 
                 int32_t vis_low, 
                 int32_t ir, 
                 LUX_COEFF *lk)
{
  int32_t lux;

  if( (vis_high > ADC_THRESHOLD) || (ir > ADC_THRESHOLD) ) 
  {
    lux = eval_poly( vis_high, 
                     ir, 
                     INPUT_FRACTION_HIGH, 
                     LUX_OUTPUT_FRACTION,
                     NUMCOEFF_HIGH,
                     &(lk->coeff_high[0]) );
  }
  else
  {
    lux = eval_poly( vis_low, 
                     ir, 
                     INPUT_FRACTION_LOW, 
                     LUX_OUTPUT_FRACTION,
                     NUMCOEFF_LOW,
                     &(lk->coeff_low[0]) );
  }
  return lux;
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
//    SI115X_SAMPLES structure and calls the get_lux()
//    routine to compute the lux
//
float lux_call_example( HANDLE si115x_handle, 
                        SI115X_SAMPLES *samples )
{
  float lux;

  //
  // Example conversion to human-readable lux values
  //
  lux = (float) get_lux( samples->ch1,
                         samples->ch3,
                         samples->ch2, 
                         &lk);
  lux = lux / ( 1 << LUX_OUTPUT_FRACTION );

  return lux;
}