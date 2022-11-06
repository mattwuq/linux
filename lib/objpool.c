// SPDX-License-Identifier: GPL-2.0

#include <linux/objpool.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/atomic.h>
#include <linux/prefetch.h>
#include <linux/cpumask.h>

/*
 * objpool: ring-array based lockless MPMC/FIFO queues
 *
 * Copyright: wuqiang.matt@bytedance.com
 */

/* compute the suitable num of objects to be managed by slot */
static inline int objpool_nobjs(int size)
{
	return rounddown_pow_of_two((size - sizeof(struct objpool_slot)) /
			(sizeof(uint32_t) + sizeof(void *)));
}

#define SLOT_AGES(s) ((uint32_t *)((char *)(s) + sizeof(struct objpool_slot)))
#define SLOT_ENTS(s) ((void **)((char *)(s) + sizeof(struct objpool_slot) + \
			sizeof(uint32_t) * (s)->size))
#define SLOT_OBJS(s) ((void *)((char *)(s) + sizeof(struct objpool_slot) + \
			(sizeof(uint32_t) + sizeof(void *)) * (s)->size))
#define SLOT_CORE(n) cpumask_nth((n) % num_possible_cpus(), cpu_possible_mask)

/* allocate and initialize percpu slots */
static inline int
objpool_init_percpu_slots(struct objpool_head *head, int nobjs,
			void *context, objpool_init_obj_cb objinit)
{
	int i, j, n, size, objsz, cpu = 0, nents = head->capacity;

	/* aligned object size by sizeof(void *) */
	objsz = ALIGN(head->obj_size, sizeof(void *));
	/* shall we allocate objects along with objpool_slot */
	if (objsz)
		head->flags |= OBJPOOL_HAVE_OBJECTS;

	for (i = 0; i < head->nr_cpus; i++) {
		struct objpool_slot *os;

		/* skip the cpus which could never be present */
		if (!cpu_possible(i))
			continue;

		/* compute how many objects to be managed by this slot */
		n = nobjs / num_possible_cpus();
		if (cpu < (nobjs % num_possible_cpus()))
			n++;
		size = sizeof(struct objpool_slot) + sizeof(void *) * nents +
		       sizeof(uint32_t) * nents + objsz * n;

		/* decide memory area for cpu-slot allocation */
		if (!cpu && !(head->gfp & GFP_ATOMIC) && size > PAGE_SIZE / 2)
			head->flags |= OBJPOOL_FROM_VMALLOC;

		/* allocate percpu slot & objects from local memory */
		if (head->flags & OBJPOOL_FROM_VMALLOC)
			os = __vmalloc_node(size, sizeof(void *), head->gfp,
				cpu_to_node(i), __builtin_return_address(0));
		else
			os = kmalloc_node(size, head->gfp, cpu_to_node(i));
		if (!os)
			return -ENOMEM;

		/* initialize percpu slot for the i-th slot */
		memset(os, 0, size);
		os->size = head->capacity;
		os->mask = os->size - 1;
		head->cpu_slots[i] = os;
		head->slot_sizes[i] = size;
		cpu = cpu + 1;

		/*
		 * start from 2nd round to avoid conflict of 1st item.
		 * we assume that the head item is ready for retrieval
		 * iff head is equal to ages[head & mask]. but ages is
		 * initialized as 0, so in view of the caller of pop(),
		 * the 1st item (0th) is always ready, but fact could
		 * be: push() is stalled before the final update, thus
		 * the item being inserted will be lost forever.
		 */
		os->head = os->tail = head->capacity;

		if (!objsz)
			continue;

		for (j = 0; j < n; j++) {
			uint32_t *ages = SLOT_AGES(os);
			void **ents = SLOT_ENTS(os);
			void *obj = SLOT_OBJS(os) + j * objsz;
			uint32_t ie = os->tail & os->mask;

			/* perform object initialization */
			if (objinit) {
				int rc = objinit(obj, context);
				if (rc)
					return rc;
			}

			/* add obj into the ring array */
			ents[ie] = obj;
			ages[ie] = os->tail;
			os->tail++;
			head->nr_objs++;
		}
	}

	return 0;
}

/* cleanup all percpu slots of the object pool */
static inline void objpool_fini_percpu_slots(struct objpool_head *head)
{
	int i;

	if (!head->cpu_slots)
		return;

	for (i = 0; i < head->nr_cpus; i++) {
		if (!head->cpu_slots[i])
			continue;
		if (head->flags & OBJPOOL_FROM_VMALLOC)
			vfree(head->cpu_slots[i]);
		else
			kfree(head->cpu_slots[i]);
	}
	kfree(head->cpu_slots);
	head->cpu_slots = NULL;
	head->slot_sizes = NULL;
}

/**
 * objpool_init: initialize object pool and pre-allocate objects
 *
 * args:
 * @head:    the object pool to be initialized, declared by caller
 * @nr_objs: total objects to be pre-allocated by this object pool
 * @object_size: size of an object, no objects pre-allocated if 0
 * @gfp:     flags for memory allocation (via kmalloc or vmalloc)
 * @context: user context for object initialization callback
 * @objinit: object initialization callback for extra setting-up
 * @release: cleanup callback for private objects/pool/context
 *
 * return:
 *         0 for success, otherwise error code
 *
 * All pre-allocated objects are to be zeroed. Caller could do extra
 * initialization in objinit callback. The objinit callback will be
 * called once and only once after the slot allocation. Then objpool
 * won't touch any content of the objects since then. It's caller's
 * duty to perform reinitialization after object allocation (pop) or
 * clearance before object reclamation (push) if required.
 */
int objpool_init(struct objpool_head *head, int nr_objs, int object_size,
		gfp_t gfp, void *context, objpool_init_obj_cb objinit,
		objpool_fini_cb release)
{
	int nents, rc;

	/* check input parameters */
	if (nr_objs <= 0 || object_size < 0)
		return -EINVAL;

	/* calculate percpu slot size (rounded to pow of 2) */
	nents = max_t(int, roundup_pow_of_two(nr_objs),
			objpool_nobjs(L1_CACHE_BYTES));

	/* initialize objpool head */
	memset(head, 0, sizeof(struct objpool_head));
	head->nr_cpus = nr_cpu_ids;
	head->obj_size = object_size;
	head->capacity = nents;
	head->gfp = gfp & ~__GFP_ZERO;
	head->context = context;
	head->release = release;

	/* allocate array for percpu slots */
	head->cpu_slots = kzalloc(head->nr_cpus * sizeof(void *) +
			       head->nr_cpus * sizeof(uint32_t), head->gfp);
	if (!head->cpu_slots)
		return -ENOMEM;
	head->slot_sizes = (uint32_t *)&head->cpu_slots[head->nr_cpus];

	/* initialize per-cpu slots */
	rc = objpool_init_percpu_slots(head, nr_objs, context, objinit);
	if (rc)
		objpool_fini_percpu_slots(head);

	return rc;
}
EXPORT_SYMBOL_GPL(objpool_init);

/* adding object to slot tail, the given slot must NOT be full */
static inline int objpool_add_slot(void *obj, struct objpool_slot *os)
{
	uint32_t *ages = SLOT_AGES(os);
	void **ents = SLOT_ENTS(os);
	uint32_t tail = atomic_inc_return((atomic_t *)&os->tail) - 1;

	WRITE_ONCE(ents[tail & os->mask], obj);

	/* order matters: obj must be updated before tail updating */
	smp_store_release(&ages[tail & os->mask], tail);
	return 0;
}

/**
 * objpool_push: reclaim the object and return back to objects pool
 *
 * args:
 * @obj:  object pointer to be pushed to object pool
 * @head: object pool
 *
 * return:
 *     0 or error code: it fails only when objects pool are full
 *
 * objpool_push is non-blockable, and can be nested
 */
int objpool_push(void *obj, struct objpool_head *head)
{
	int cpu = raw_smp_processor_id();

	return objpool_add_slot(obj, head->cpu_slots[cpu]);
}
EXPORT_SYMBOL_GPL(objpool_push);

/* try to retrieve object from slot */
static inline void *objpool_try_get_slot(struct objpool_slot *os)
{
	uint32_t *ages = SLOT_AGES(os);
	void **ents = SLOT_ENTS(os);
	/* do memory load of head to local head */
	uint32_t head = smp_load_acquire(&os->head);

	/* loop if slot isn't empty */
	while (head != READ_ONCE(os->tail)) {
		uint32_t id = head & os->mask, prev = head;

		/* do prefetching of object ents */
		prefetch(&ents[id]);

		/*
		 * check whether this item was ready for retrieval ? There's
		 * possibility * in theory * we might retrieve wrong object,
		 * in case ages[id] overflows when current task is sleeping,
		 * but it will take very very long to overflow an uint32_t
		 */
		if (smp_load_acquire(&ages[id]) == head) {
			/* node must have been udpated by push() */
			void *node = READ_ONCE(ents[id]);
			/* commit and move forward head of the slot */
			if (try_cmpxchg_release(&os->head, &head, head + 1))
				return node;
		}

		/* re-load head from memory and continue trying */
		head = READ_ONCE(os->head);
		/*
		 * head stays unchanged, so it's very likely current pop()
		 * just preempted/interrupted an ongoing push() operation
		 */
		if (head == prev)
			break;
	}

	return NULL;
}

/**
 * objpool_pop: allocate an object from objects pool
 *
 * args:
 * @head: object pool
 *
 * return:
 *   object: NULL if failed (object pool is empty)
 *
 * objpool_pop can be nested, so can be used in any context.
 */
void *objpool_pop(struct objpool_head *head)
{
	int i, cpu = raw_smp_processor_id();
	void *obj = NULL;

	for (i = 0; i < num_possible_cpus(); i++) {
		obj = objpool_try_get_slot(head->cpu_slots[cpu]);
		if (obj)
			break;
		cpu = cpumask_next_wrap(cpu, cpu_possible_mask, -1, 1);
	}

	return obj;
}
EXPORT_SYMBOL_GPL(objpool_pop);

/**
 * objpool_fini: cleanup the whole object pool (releasing all objects)
 *
 * args:
 * @head: object pool to be released
 *
 */
void objpool_fini(struct objpool_head *head)
{
	if (!head->cpu_slots)
		return;

	/* release percpu slots */
	objpool_fini_percpu_slots(head);

	/* call user's cleanup callback if provided */
	if (head->release)
		head->release(head, head->context);
}
EXPORT_SYMBOL_GPL(objpool_fini);
