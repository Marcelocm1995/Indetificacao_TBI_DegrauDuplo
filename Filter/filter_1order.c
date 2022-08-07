#include "filter_1order.h"

/*
 * Parameters:          LfFiltred_Old       Current value to be filtered
 *                      LfRawVal       New input value
 *                      LfFilterFactor   Filter Coefficient
 *
 *****************************************************************************/
double FilterDoubleVal(double LfFiltred_Old, double LfRawVal, double LfFilterFactor)
{
  double LfFilterResult;

  if ( (LfFiltred_Old == LfRawVal)
     || (LfFilterFactor == 0) )
      {
         LfFilterResult = LfFiltred_Old;
      }

  else if (LfRawVal > LfFiltred_Old)
      {
          LfFilterResult = LfRawVal - LfFiltred_Old;
          LfFilterResult = LfFilterFactor * LfFilterResult;
          LfFilterResult = LfFiltred_Old + LfFilterResult;
      }

  else /* if (LfRawVal < LfFiltred_Old) */
      {
          LfFilterResult = LfFiltred_Old - LfRawVal;
          LfFilterResult = LfFilterFactor * LfFilterResult;
          LfFilterResult = LfFiltred_Old - LfFilterResult;
      } /* end if (LfFiltred_Old > LfRawVal) */

  return(LfFilterResult);
} /* End ApplyFILT_1stOrderLag_usp */

float FilterFloatVal(float LfFiltred_Old, float LfRawVal, float LfFilterFactor)
{
  float LfFilterResult;

  if ( (LfFiltred_Old == LfRawVal)
     || (LfFilterFactor == 0) )
      {
         LfFilterResult = LfFiltred_Old;
      }

  else if (LfRawVal > LfFiltred_Old)
      {
          LfFilterResult = LfRawVal - LfFiltred_Old;
          LfFilterResult = LfFilterFactor * LfFilterResult;
          LfFilterResult = LfFiltred_Old + LfFilterResult;
      }

  else /* if (LfRawVal < LfFiltred_Old) */
      {
          LfFilterResult = LfFiltred_Old - LfRawVal;
          LfFilterResult = LfFilterFactor * LfFilterResult;
          LfFilterResult = LfFiltred_Old - LfFilterResult;
      } /* end if (LfFiltred_Old > LfRawVal) */

  return(LfFilterResult);
} /* End ApplyFILT_1stOrderLag_usp */
