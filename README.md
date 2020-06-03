# rtos-on-tiva
Real time operating system from scratch on TI board (_**academice project**_)

## Overview:
This is an academic project. The goal of this project is to write a simple RTOS for an M4F controller that implements a selectable cooperative or preemptive RTOS with support for semaphores, yielding, and system timers. 

A simple framework for building the RTOS is included in the rtos.c file.

## Requirements:
### Scheduler:
* Modify the rtos.c functions to support system timers, yielding, and semaphores.
* Each time the scheduler is called, it will look at all ready processes and will choose the next process. Modify the scheduler to add prioritization to 8 levels.
* Note: The method used to prioritize the relative importance of the tasks is implicit in the assignment of the prioritization when createThread() is called.
### Kernel Functions:
* Add a function yield() that will yield execution back to the kernel that will store the return address and save the context necessary for the resuming the process later.
* Add a function sleep(time_ms) and supporting kernel code that will store the return address and save the context necessary for the resuming the process later. The process is then delayed until a kernel determines that a period of time_ms has expired. Once the time has expired, the process that called sleep(time_ms) will be marked as ready so that the scheduler can resume the process later.
* Add a function wait(semaphore) that causes a process to block until a resource or resources is available. The waiting process will be recorded in the semaphore process queue.
* Add a function post(semaphore) and supporting kernel code as discussed in the lectures.
* Modify the function createThread() to store the process name and initialize the process stack as needed.
* Add a function destroyProcess() that removes a process from the TCB and cleans up all semaphore entries for the process that is deleted.
* Code the function systickIsr() that handles the sleep timing and handles the preemptive task switching.
* Code the function svcallIsr() the handles all task switching for preemptive mode. All preemptive task switching needs to use this ISR. Modify the createThread(), deleteProcess(), rtosStart(), yield(), sleep(), wait(), and post() to use this SVC ISR.
* Add a shell process that hosts a command line interface to the PC. The command-line interface should support the following commands (borrowing from UNIX):

  - ps: The PID id, process name, and % of CPU time should be stored at a minimum.
  - ipcs: At a minimum, the semaphore usage should be displayed.
  - kill <PID>:This command allows a task to be killed, by referencing the process ID.
  - reboot: The command restarted the processor.
  - pidof <Process_Name> returns the PID of a process
  - <Process_Name> & starts a process running in the background if not already running. Only one instance of a named process is allowed.
