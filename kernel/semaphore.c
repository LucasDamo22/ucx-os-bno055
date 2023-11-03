/* file:          semaphore.c
 * description:   semaphore implementation
 * date:          11/2021
 * author:        Sergio Johann Filho <sergio.johann@acad.pucrs.br>
 */

#include <ucx.h>

struct sem_s *ucx_sem_create(uint16_t max_tasks, int32_t value)
{
	struct sem_s *s;
	
	s = (struct sem_s *)malloc(sizeof(struct sem_s));
	
	if (!s)
		return 0;
	
	s->sem_queue = queue_create(max_tasks);
	
	if ((!s->sem_queue) || (value < 0)) {
		free(s);
		
		return 0;
	} else {
		s->count = value;

		return s;
	}
}

int32_t ucx_sem_destroy(struct sem_s *s)
{
	
	if (queue_destroy(s->sem_queue)) {
		return -1;
	} else {
		free(s);
		
		return 0;
	}
}

void ucx_sem_wait(struct sem_s *s)
{
	struct tcb_s *tcb_sem;
	
	CRITICAL_ENTER();
	s->count--;
	if (s->count < 0) {
		tcb_sem = kcb->task_current->data;
		queue_enqueue(s->sem_queue, tcb_sem);
		tcb_sem->state = TASK_BLOCKED;
		CRITICAL_LEAVE();
		ucx_task_wfi();
	} else {
		CRITICAL_LEAVE();
	}
}

void ucx_sem_signal(struct sem_s *s)
{
	struct tcb_s *tcb_sem;
	
	CRITICAL_ENTER();
	s->count++;
	if (s->count <= 0) {
		tcb_sem = queue_dequeue(s->sem_queue);
		tcb_sem->state = TASK_READY;
	}
	CRITICAL_LEAVE();
}
