/*
 * util.h
 *
 *  Created on: 8 февр. 2025 г.
 *      Author: bryki
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#define _SEC_IN_MINUTE 60L
#define _SEC_IN_HOUR 3600L
#define _SEC_IN_DAY 86400L

static const int DAYS_IN_MONTH[12] =
{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

#define _DAYS_IN_MONTH(x) ((x == 1) ? days_in_feb : DAYS_IN_MONTH[x])

static const int _DAYS_BEFORE_MONTH[12] =
{0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define _ISLEAP(y) (((y) % 4) == 0 && (((y) % 100) != 0 || (((y)+1900) % 400) == 0))
#define _DAYS_IN_YEAR(year) (_ISLEAP(year) ? 366 : 365)

uint32_t board_ConvertToUnixTime(RTC_DateTypeDef *psDate, RTC_TimeTypeDef *psTime)
{
    uint32_t tim = 0;
    long days = 0;
    int year = 0;

    psDate->Month--;

    psDate->Year += 100;

    /* compute hours, minutes, seconds */
    tim += psTime->Seconds + (psTime->Minutes * _SEC_IN_MINUTE) + (psTime->Hours * _SEC_IN_HOUR);

    /* compute days in year */
    days += psDate->Date - 1;
    days += _DAYS_BEFORE_MONTH[psDate->Month];

    if (psDate->Month > 1 && _DAYS_IN_YEAR (psDate->Year) == 366)
        days++;

    /* compute days in other years */
    if ((year = psDate->Year) > 70)
    {
      for (year = 70; year < psDate->Year; year++)
          days += _DAYS_IN_YEAR (year);
    }
    else if (year < 70)
    {
      for (year = 69; year > psDate->Year; year--)
          days -= _DAYS_IN_YEAR (year);

      days -= _DAYS_IN_YEAR (year);
    }

    /* compute total seconds */
    tim += (uint32_t )days * _SEC_IN_DAY;

    return tim;
}


#endif /* INC_UTIL_H_ */
