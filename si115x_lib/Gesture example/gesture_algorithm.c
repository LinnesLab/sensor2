/**************************************************************************//**
 * @brief
  *  Implements the algorithm for detecting gestures on the sensor STK.
  *  Should be called with new sample data every time an interrupt is
  *  received.
 * @param[in] samples
 *   New sample data received from the sensor.
 * @return
 *   Returns the type of gesture detected (as defined by gesture_t).
 *****************************************************************************/
#include "si115x_functions.h"

typedef enum
{
  NONE,
  UP,
  DOWN,
  LEFT,
  RIGHT,
  PROX
} gesture_t;


typedef struct
{
  u32 timestamp;         /* Timestamp to record */
  u16 ps1;               /* PS1 */
  u16 ps2;               /* PS2 */
  u16 ps3;               /* PS3 */
} Si115x_Sample_TypeDef;

gesture_t GestureAlgorithm(Si115x_Sample_TypeDef *samples)
{
  u16        ps[3];

  static u32 ps_entry_time[3] = { 0, 0, 0 };
  static u32 ps_exit_time[3]  = { 0, 0, 0 };

  static u8  ps_state[3] = { 0, 0, 0 };

  u8         array_counter;
  u32 diff_x ;
  u32 diff_y1 ;
  u32 diff_y2 ;
  u32 ps_time[3] ;
  u32 ps_avg;
  gesture_t  ret = NONE;  /*gesture result return value */
  /*save new samples into ps array */
  ps[0] = samples->ps1;
  ps[1] = samples->ps2;
  ps[2] = samples->ps3;

  /* Check state of all three measurements */
  for (array_counter = 0; array_counter < 3; array_counter++)
  {
    /* If measurement higher than the ps_threshold value, */
    /*   record the time of entry and change the state to look for the exit time */
    if (ps[array_counter] >= PS_THRESHOLD)
    {
      ret = PROX; 
      if (ps_state[array_counter] == 0)
      {
        ps_state[array_counter]      = 1;
        ps_entry_time[array_counter] = samples->timestamp;
      }
    }
    else
    {
      if (ps_state[array_counter] == 1)
      {
        ps_state[array_counter]     = 0;
        ps_exit_time[array_counter] = samples->timestamp;
      }
    }
  }

  /* If there is no object in front of the board, look at history to see if a gesture occured */
  if ((ps[0] < PS_THRESHOLD) && (ps[1] < PS_THRESHOLD) && (ps[2] < PS_THRESHOLD))
  {
    /* If the ps_max values are high enough and there exit entry and exit times, */
    /*   then begin processing gestures */
    if ((ps_entry_time[0] != 0) && (ps_entry_time[1] != 0) && (ps_entry_time[2] != 0)
        && (ps_exit_time[0] != 0) && (ps_exit_time[1] != 0) && (ps_exit_time[2] != 0))
    {
      /* Make sure no timestamps overflowed, indicated possibility if any of them are close to overflowing */
      if ((ps_exit_time[0] > 0xFC000000L) || (ps_exit_time[1] > 0xFC000000L) || (ps_exit_time[2] > 0xFC000000L)
          || (ps_entry_time[0] > 0xFC000000L) || (ps_entry_time[1] > 0xFC000000L) || (ps_entry_time[2] > 0xFC000000L))
      {         /* If any of them are close to overflowing, overflow them all so they all have the same reference */
        ps_exit_time[0] += 0x1FFFFFFFL;
        ps_exit_time[1] += 0x1FFFFFFFL;
        ps_exit_time[2] += 0x1FFFFFFFL;

        ps_entry_time[0] += 0x1FFFFFFFL;
        ps_entry_time[1] += 0x1FFFFFFFL;
        ps_entry_time[2] += 0x1FFFFFFFL;
      }

      /* Calculate the midpoint (between entry and exit times) of each waveform */
      /*  the order of these midpoints helps determine the gesture */
      ps_time[0] = (ps_exit_time[0] - ps_entry_time[0]) / 2;
      ps_time[0] = ps_time[0] + ps_entry_time[0];

      ps_time[1] = (ps_exit_time[1] - ps_entry_time[1]) / 2;
      ps_time[1] = ps_time[1] + ps_entry_time[1];

      ps_time[2] = (ps_exit_time[2] - ps_entry_time[2]) / 2;
      ps_time[2] = ps_time[2] + ps_entry_time[2];

      /* The diff_x and diff_y values help determine a gesture by comparing the */
      /*  LED measurements that are on a single axis */
      if (ps_time[1] > ps_time[2])
      {
        diff_x = ps_time[1] - ps_time[2];
      }
      else
      {
        diff_x = ps_time[2] - ps_time[1];
      }
      if( ps_time[0] > ps_time[1] )
      {
        diff_y1 = ps_time[0] - ps_time[1];
      }
	  else
      {
        diff_y1 = ps_time[1] - ps_time[0];
      }

      if( ps_time[0] > ps_time[2] )
      {
        diff_y2 = ps_time[0] - ps_time[2];
      }
	  else
      {
        diff_y2 = ps_time[2] - ps_time[0];
      }


      /* Take the average of all three midpoints to make a comparison point for each midpoint */
      ps_avg = (u32) ps_time[0] + (u32) ps_time[1] + (u32) ps_time[2];
      ps_avg = ps_avg / 3;

      if ((ps_exit_time[0] - ps_entry_time[0]) > 10 || (ps_exit_time[1] - ps_entry_time[1]) > 10 || (ps_exit_time[2] - ps_entry_time[2]) > 10)
      {
        if( ( (ps_time[0] < ps_time[1]) &&  (diff_y1 > diff_x) ) || ( (ps_time[0] <= ps_time[2]) && (diff_y2 > diff_x) ) )
        {           /* An up gesture occured if the bottom LED had its midpoint first */
          ret = UP;
        }
        else if  ( ( (ps_time[0] < ps_time[1]) &&  (diff_y1 > diff_x) ) || ( (ps_time[0] > ps_time[2]) && (diff_y2 > diff_x) ) )
        {           /* A down gesture occured if the bottom LED had its midpoint last */
          ret = DOWN;
        }
        else if((ps_time[0] < ps_time[1]) && (ps_time[2] < ps_time[1]) && (diff_x > ((diff_y1+diff_y2)/2)))
        {           /* A left gesture occured if the left LED had its midpoint last */
          ret = LEFT;
        }
        else if( (ps_time[0] < ps_time[2]) && (ps_time[1] < ps_time[2])  && (diff_x > ((diff_y1+diff_y2)/2)))
        {           /* A right gesture occured if the right LED had midpoint later than the right LED */
          ret = RIGHT;
        }
      }
    }
    for (array_counter = 0; array_counter < 3; array_counter++)
    {
      ps_exit_time[array_counter]  = 0;
      ps_entry_time[array_counter] = 0;
    }
  }

  return ret;
}