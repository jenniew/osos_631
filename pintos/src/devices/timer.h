/*
 * timer.h
 *
 *  Created on: Nov 8, 2014
 *      Author: jcyescas
 */

#ifndef DEVICES_TIMER_H_
#define DEVICES_TIMER_H_

#include "../threads/synch.h"

struct timer_wait_node{
    struct semaphore sem;
    struct list_elem elem;
    struct thread *t;
};

void timer_init(void);

int timer_get_timestamp();

void timer_msleep(int milliseconds);

#endif /* TIMER_H_ */
