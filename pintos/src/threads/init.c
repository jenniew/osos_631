
#include <debug.h>
#include <random.h>
#include <stdbool.h>
#include "stdio.h"
#include <stdint.h>
#include <string.h>

#include "../devices/gpio.h"
#include "../devices/framebuffer.h"
#include "../devices/serial.h"
#include "../devices/timer.h"
#include "../devices/video.h"
#include "interrupt.h"
#include "init.h"
#include "palloc.h"
#include "malloc.h"
#include "synch.h"
#include "thread.h"
#include "vaddr.h"

#include "../lib/string.h"

/* -ul: Maximum number of pages to put into palloc's user pool. */
static size_t user_page_limit = SIZE_MAX;

/* Tasks for the Threads. */
static struct semaphore task_sem;


static void hello_test(void *);
static void A();
static void B();
static void C();
static void command_line();
static void command_help();
static void command_ts();

struct wait_node {
    struct lock mutex;
    struct condition cv;
    bool done;
};
static struct wait_node sync_node;

static void t_wait(struct wait_node *wn);
static void t_exit(struct wait_node *wn);

static void cv_test(void *);
//static void cv_test(void *);
static void shell(void *);



/*
 * kernel.c
 *
 *  Created on: Oct 22, 2014
 *      Author: jcyescas
 */

static void test_swi_interrupt() {
  unsigned short green = 0x7E0;
  SetForeColour(green);
  generate_swi_interrupt(); // Function defined in interrupts.s
}

static int tid_b;

/* Initializes the Operating System. The interruptions have to be disabled at entrance.
*
*  - Sets interruptions
*  - Sets the periodic timer
*  - Set the thread functionality
*/
void init() {


    /* Initializes ourselves as a thread so we can use locks,
      then enable console locking. */
    thread_init();

    /* Initializes the frame buffer and console. */
    framebuffer_init();
    serial_init();
    video_init();

    printf("\nOsOs Kernel Initializing");

    /* Initialize memory system. */
    palloc_init (user_page_limit);
    malloc_init ();

    /* Initializes the Interrupt System. */
    interrupts_init();
    timer_init();
    keyboard_init();

    timer_msleep(5000000);

    /* Starts preemptive thread scheduling by enabling interrupts. */
    thread_start();

    printf("\nFinish booting.");

    /* Initialize the task_sem to coordinate the main thread with workers */

    sema_init(&task_sem, 0);

//    thread_create("Hello Test", PRI_MAX, &hello_test, NULL);
//    sema_down(&task_sem);
//    thread_create("A", 35, &A, NULL);
//    thread_create("B", 36, &B, NULL);
//    thread_create("C", 37, &C, NULL);
    thread_create("shell", PRI_MAX, &command_line, NULL);
//    thread_create("print thread",37,&command_ts,NULL);
    lock_init(&sync_node.mutex);
    cond_init(&sync_node.cv);
    sync_node.done = false;

//    thread_create("CV Test", PRI_MAX, &cv_test, NULL);
//    t_wait(&sync_node);


    printf("\nAll done.");
    thread_exit ();
}

static void hello_test(void *aux) {
    printf("\n");
    printf("Hello from OsOS\n");
    printf("\n");
}

static void t_wait(struct wait_node *wn)
{
    lock_acquire(&wn->mutex);
    while (!wn->done) {
        cond_wait(&wn->cv, &wn->mutex);
    }
    lock_release(&wn->mutex);
}

static void t_exit(struct wait_node *wn)
{
    lock_acquire(&wn->mutex);
    wn->done = true;
    cond_signal(&wn->cv,&wn->mutex);
    lock_release(&wn->mutex);
}

static void cv_test(void *aux) {
    printf("\n");
    printf("Hello from CV Test\n");
    printf("\n");
    t_exit(&sync_node);
}

static void A(){
    int i = 0;
    for (i = 0; i < 50; ++i) {
        printf("\nAAAAAAAAAAAAAAAAAAAAAAAAAA\n");
    }
}

static void B(){
    int i = 0;
    for (i = 0; i < 50; ++i) {
        printf("\nBBBBBBBBBBBBBBBBBBBBBBBBBB\n");
    }
}

static void C(){
//    int i = 0;
//    for (i = 0; i < 50; ++i) {
//        printf("CCCCCCCCCCCCCCCCCCCCCCCCCC\n");
//
//    }
    while (1){

    }

}

void command_line(){
    int i=0,j=0,k=0,l=0,m=0;
    char buf[100],buf1[100],buf2[100],func_name[100];
    memset(buf,0,100);
    memset(func_name,0,100);
    char *ts = "ts";
    char *help = "help";
    char *run = "run ";
    char *bg = "bg ";
    char temp;
    printf("\n\n$");
    while (1){
        temp =  uart_getc();
        uart_putc(temp);
        if (temp!='\r'){
            buf[i++] = temp;
        } else{
            j = strcmp(buf,ts);
            k = strcmp(buf,help);
            strlcpy(buf1,buf,5);
            l = strcmp(buf1,run);
            strlcpy(buf2,buf,4);
            m = strcmp(buf2,bg);
            if (j==0){        //if it is a ts call
                command_ts();
            } else if (k==0){ //if it is a help call
                command_help();
            } else if (m==0){//if it is a bg call
                const char* func = buf+3;
                strlcpy(func_name,func,strlen(func)+1);
                command_bg(func_name);
            } else if (l==0){ //if it is a run call
                const char* func = buf+4;
                strlcpy(func_name,func,strlen(func)+1);
                command_run(func_name);
            } else{
                printf("wrong command\n$");
            }
            i=0;
            memset(buf,0,100);
        }
    }
}

void command_help(){
    printf("ts --- Show threads information that tid,thread name,status\n");
    printf("run <func>---launch a thread function and wait for completion\n");
    printf("bg  <func>---launch a command in the background\n");
    printf("$");
}

void command_ts(){
    int count = getNumberOfThreads();
    printf("There are %d threads\n",count);
    thread_information_print();
    printf("$");
}


static void shell(void *aux){
  printf("\nWelcome to raspberry pi");
  printf("$");
  unsigned char buf[128];
  unsigned char* b;
  int i = 0;
  while(1){
    b = uart_gets();
  }
}

void command_bg(const char *str){
    if (strcmp(str,"A")==0){
        thread_create(str,PRI_DEFAULT,&A,NULL);
    } else if (strcmp(str,"B")==0){
        thread_create(str,PRI_DEFAULT,&B,NULL);
    } else if (strcmp(str,"C")==0){
        thread_create(str,PRI_DEFAULT,&C,NULL);
    } else if (strcmp(str,"hello_test")==0){
        thread_create(str, PRI_MAX, &hello_test, NULL);
    }
    printf("$");

}

void command_run(const char *str){
    if (strcmp(str,"A")==0){
        tid_t tid = thread_create(str,PRI_DEFAULT,&A,NULL);
        thread_wait(tid);
    } else if (strcmp(str,"B")==0){
        tid_t tid = thread_create(str,PRI_DEFAULT,&B,NULL);
        thread_wait(tid);
    } else if (strcmp(str,"C")==0){
        tid_t tid = thread_create(str,PRI_DEFAULT,&C,NULL);
        thread_wait(tid);
    } else if (strcmp(str,"hello_test")==0){
        tid_t tid = thread_create(str, PRI_MAX, &hello_test, NULL);
        thread_wait(tid);
    }
    printf("$");
}




