/* file:          ucx.c
 * description:   UCX/OS kernel
 * date:          04/2021
 * author:        Sergio Johann Filho <sergio.johann@acad.pucrs.br>
 */

#include <ucx.h>

struct kcb_s kernel_state;
struct kcb_s *kcb_p = &kernel_state;

static void guard_check(void)
{
	uint32_t check = 0x33333333;
	
	if (*kcb_p->tcb_p->guard_addr != check) {
		hexdump((void *)kcb_p->tcb_p->guard_addr, kcb_p->tcb_p->guard_sz);
		_printf("\n*** HALT - task %d, guard %08x (%d) check failed\n", kcb_p->tcb_p->id,
			(size_t)kcb_p->tcb_p->guard_addr, (size_t)kcb_p->tcb_p->guard_sz);
		for (;;);
	}
		
}

static void update_delay(void)
{
	struct tcb_s *tcb_ptr = kcb_p->tcb_first;
	
	for (;;	tcb_ptr = tcb_ptr->tcb_next) {
		if (tcb_ptr->state == TASK_BLOCKED && tcb_ptr->delay > 0) {
			tcb_ptr->delay--;
			if (tcb_ptr->delay == 0)
				tcb_ptr->state = TASK_READY;
		}
		if (tcb_ptr->tcb_next == kcb_p->tcb_first) break;
	}
}

void dispatcher(void)
{
	if (!setjmp(kcb_p->tcb_p->context)) {
		update_delay();
		guard_check();
		if (kcb_p->tcb_p->state == TASK_RUNNING)
			kcb_p->tcb_p->state = TASK_READY;
		do {
			kcb_p->tcb_p = kcb_p->tcb_p->tcb_next;
		} while (kcb_p->tcb_p->state != TASK_READY);
		kcb_p->tcb_p->state = TASK_RUNNING;
		kcb_p->ctx_switches++;
		_interrupt_tick();
		longjmp(kcb_p->tcb_p->context, 1);
	}
}

static void sched_init(int32_t preemptive)
{
	kcb_p->tcb_p = kcb_p->tcb_first;
	if (preemptive) {
		_timer_enable();
	}
	(*kcb_p->tcb_p->task)();
}


/* task management */

int32_t ucx_task_add(void *task, uint16_t guard_size)
{
	struct tcb_s *tcb_last = kcb_p->tcb_p;
	
	kcb_p->tcb_p = (struct tcb_s *)malloc(sizeof(struct tcb_s));
	if (kcb_p->tcb_first == 0)
		kcb_p->tcb_first = kcb_p->tcb_p;

	if (!kcb_p->tcb_p)
		return -1;

	if (tcb_last)
		tcb_last->tcb_next = kcb_p->tcb_p;
	kcb_p->tcb_p->tcb_next = kcb_p->tcb_first;
	kcb_p->tcb_p->task = task;
	kcb_p->tcb_p->delay = 0;
	kcb_p->tcb_p->guard_sz = guard_size;
	kcb_p->tcb_p->id = kcb_p->id++;
	kcb_p->tcb_p->state = TASK_STOPPED;
	
	return 0;
}

/*
 * First following lines of code are absurd at best. Stack marks are
 * used by guard_check() to detect stack overflows (sometimes). It is
 * up to the user to define sufficient stack guard space (considering
 * local thread allocation of the stack for recursion and context
 * saving). We also need the safety pig, just in case.
                         _ 
 _._ _..._ .-',     _.._(`)) 
'-. `     '  /-._.-'    ',/ 
   )         \            '. 
  / _    _    |             \ 
 |  a    a    /              | 
 \   .-.                     ;   
  '-('' ).-'       ,'       ; 
     '-;           |      .' 
        \           \    / 
        | 7  .__  _.-\   \ 
        | |  |  ``/  /`  / 
       /,_|  |   /,_/   / 
          /,_/      '`-' 
*/
void ucx_task_init(void)
{
	char guard[kcb_p->tcb_p->guard_sz];
	
	_memset(guard, 0x69, kcb_p->tcb_p->guard_sz);
	_memset(guard, 0x33, 4);
	_memset((guard) + kcb_p->tcb_p->guard_sz - 4, 0x33, 4);
	kcb_p->tcb_p->guard_addr = (uint32_t *)guard;
	_printf("task %d, guard: %08x - %08x\n", kcb_p->tcb_p->id, (size_t)kcb_p->tcb_p->guard_addr,
		(size_t)kcb_p->tcb_p->guard_addr + kcb_p->tcb_p->guard_sz);
	
	if (!setjmp(kcb_p->tcb_p->context)) {
		kcb_p->tcb_p->state = TASK_READY;
		if (kcb_p->tcb_p->tcb_next == kcb_p->tcb_first) {
			kcb_p->tcb_p->state = TASK_RUNNING;
		} else {
			kcb_p->tcb_p = kcb_p->tcb_p->tcb_next;
			kcb_p->tcb_p->state = TASK_RUNNING;
			(*kcb_p->tcb_p->task)();
		}
	}
	_ei(1);
}

void ucx_task_yield()
{
	if (!setjmp(kcb_p->tcb_p->context)) {
		update_delay();
		guard_check();
		if (kcb_p->tcb_p->state == TASK_RUNNING)
			kcb_p->tcb_p->state = TASK_READY;
		do {
			kcb_p->tcb_p = kcb_p->tcb_p->tcb_next;
		} while (kcb_p->tcb_p->state != TASK_READY);
		kcb_p->tcb_p->state = TASK_RUNNING;
		kcb_p->ctx_switches++;
		longjmp(kcb_p->tcb_p->context, 1);
	}
}

void ucx_task_delay(uint16_t ticks)
{
	kcb_p->tcb_p->delay = ticks;
	kcb_p->tcb_p->state = TASK_BLOCKED;
	ucx_task_yield();
}

int32_t ucx_task_suspend(uint16_t id)
{
	struct tcb_s *tcb_ptr = kcb_p->tcb_first;
	
	for (;; tcb_ptr = tcb_ptr->tcb_next) {
		if (tcb_ptr->id == id) {
			ucx_enter_critical();
			if (tcb_ptr->state == TASK_READY || tcb_ptr->state == TASK_RUNNING) {
				tcb_ptr->state = TASK_SUSPENDED;
				ucx_leave_critical();
				break;
			} else {
				ucx_leave_critical();
				return -1;
			}
		}
		if (tcb_ptr->tcb_next == kcb_p->tcb_first)
			return -1;
	}
	if (kcb_p->tcb_p->id == id)
		ucx_task_yield();
	
	return 0;
}

int32_t ucx_task_resume(uint16_t id)
{
	struct tcb_s *tcb_ptr = kcb_p->tcb_first;
	
	for (;; tcb_ptr = tcb_ptr->tcb_next) {
		if (tcb_ptr->id == id) {
			ucx_enter_critical();
			if (tcb_ptr->state == TASK_SUSPENDED) {
				tcb_ptr->state = TASK_READY;
				ucx_leave_critical();
				break;
			} else {
				ucx_leave_critical();
				return -1;
			}
		}	
		if (tcb_ptr->tcb_next == kcb_p->tcb_first)
			return -1;
	}
	if (kcb_p->tcb_p->id == id)
		ucx_task_yield();
	
	return 0;
}


uint16_t ucx_task_id()
{
	return kcb_p->tcb_p->id;
}

void ucx_task_wfi()
{
	volatile uint32_t s;
	
	s = kcb_p->ctx_switches;
	while (s == kcb_p->ctx_switches);
}

uint16_t ucx_tasks()
{
	return kcb_p->id + 1;
}

void ucx_enter_critical()
{
	_timer_disable();
}

void ucx_leave_critical()
{
	_timer_enable();
}


int32_t main(void)
{
	int32_t pr;
	
	_hardware_init();
	
	kcb_p->tcb_p = 0;
	kcb_p->tcb_first = 0;
	kcb_p->ctx_switches = 0;
	kcb_p->id = 0;
	
	_printf("UCX/OS boot on %s\n", __ARCH__);
#ifndef UCX_OS_HEAP_SIZE
	ucx_heap_init((size_t *)&_heap_start, (size_t)&_heap_size);
	_printf("heap_init(), %d bytes free\n", (size_t)&_heap_size);
#else
	ucx_heap_init((size_t *)&_heap, UCX_OS_HEAP_SIZE);
	_printf("heap_init(), %d bytes free\n", UCX_OS_HEAP_SIZE);
#endif
	pr = app_main();
	sched_init(pr);
	
	return 0;
}
