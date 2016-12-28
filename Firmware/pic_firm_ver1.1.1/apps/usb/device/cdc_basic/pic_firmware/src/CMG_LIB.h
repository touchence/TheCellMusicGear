/* 
 * File:   CMG_LIB.h
 * Author: Touchence inc.
 *
 * Created on 2016/12/14, 14:00
 */
#include<stdint.h>
#ifndef CMG_LIB_H
#define	CMG_LIB_H

#ifdef	__cplusplus
extern "C" {
#endif
/****************************************************************************
  Function:
     float GetCoef(int offset);

  Description:
   This function is used to get the coefficient. The output obtained from
   each sensor is unbalanced. This coefficient is used to adjust a balance of
   these output.

  Parameters:
   Sensor's offset output. 

  Return Values:
  Coefficient

  Remarks:
    None
  ***************************************************************************/

    float GetCoef(uint16_t offset);


#ifdef	__cplusplus
}
#endif

#endif	/* CMG_LIB_H */

