#ifndef __CLOCK_H
#define __CLOCK_H

/**
 * @file clock.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "clock" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

/**
 * @brief Initialize and start the High Frequency clock
 */
void db_hfclk_init(void);

/**
 * @brief Initialize and start the Low Frequency clock
 */
void db_lfclk_init(void);

#endif
