/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_TIMEX_H
#define __ASM_TIMEX_H

<<<<<<< HEAD
<<<<<<< HEAD
=======
#include <asm/arch_timer.h>

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
/*
 * Use the current timer as a cycle counter since this is what we use for
 * the delay loop.
 */
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
#define get_cycles()	({ cycles_t c; read_current_timer(&c); c; })

#include <asm-generic/timex.h>

#define ARCH_HAS_READ_CURRENT_TIMER

<<<<<<< HEAD
=======
#define get_cycles()	arch_counter_get_cntvct()

#include <asm-generic/timex.h>

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
#endif
