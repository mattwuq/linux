/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _LINUX_OBJPOOL_H
#define _LINUX_OBJPOOL_H

#include <linux/types.h>

/*
 * objpool: ring-array based lockless MPMC queue
 *
 * Copyright: wuqiang.matt@bytedance.com
 *
 * The object pool is a scalable implementaion of high performance queue
 * for objects allocation and reclamation, such as kretprobe instances.
 *
 * With leveraging per-cpu ring-array to mitigate the hot spots of memory
 * contention, it could deliver near-linear scalability for high parallel
 * scenarios. The ring-array is compactly managed in a single cache-line
 * to benefit from warmed L1 cache for most cases (<= 4 objects per-core).
 * The body of pre-allocated objects is stored in continuous cache-lines
 * just after the ring-array.
 *
 * The object pool is interrupt safe. Both allocation and reclamation
 * (object pop and push operations) can be preemptible or interruptable.
 *
 * It's best suited for following cases:
 * 1) Memory allocation or reclamation are prohibited or too expensive
 * 2) Consumers are of different priorities, such as irqs and threads
 *
 * Limitations:
 * 1) Maximum objects (capacity) is determined during pool initializing
 * 2) The memory of objects won't be freed until the poll is finalized
 * 3) Object allocation (pop) may fail after trying all cpu slots
 */

/*
 * objpool_slot: per-cpu ring array
 *
 * Represents a cpu-local array-based ring buffer, its size is specialized
 * during initialization of object pool.
 *
 * The objpool_slot is allocated from local memory for NUMA system, and to
 * be kept compact in a single cacheline. ages[] is stored just after the
 * body of objpool_slot, and then entries[]. The Array of ages[] describes
 * revision of each item, solely used to avoid ABA. And array of entries[]
 * contains the pointers of objects.
 *
 * The default size of objpool_slot is a single cache-line, aka. 64 bytes.
 *
 * 64bit:
 *        4      8      12     16        32                 64
 * | head | tail | size | mask | ages[4] | ents[4]: (8 * 4) | objects
 *
 * 32bit:
 *        4      8      12     16        32        48       64
 * | head | tail | size | mask | ages[4] | ents[4] | unused | objects
 *
 */

struct objpool_slot {
	uint32_t                head;	/* head of ring array */
	uint32_t                tail;	/* tail of ring array */
	uint32_t                size;	/* array size, pow of 2 */
	uint32_t                mask;	/* size - 1 */
} __packed;

struct objpool_head;

/* caller-specified callback for object initial setup, only called once */
typedef int (*objpool_init_obj_cb)(void *obj, void *context);

/* caller-specified cleanup callback for objpool destruction */
typedef int (*objpool_fini_cb)(struct objpool_head *head, void *context);

/*
 * objpool_head: object pooling metadata
 */

struct objpool_head {
	int                     obj_size;	/* object & element size */
	int                     nr_objs;	/* total objs (to be pre-allocated) */
	int                     nr_cpus;	/* nr_cpu_ids */
	int                     capacity;	/* max objects per cpuslot */
	gfp_t                   gfp;		/* gfp flags for kmalloc & vmalloc */
	unsigned long           flags;		/* flags for objpool management */
	struct objpool_slot   **cpu_slots;	/* array of percpu slots */
	int                    *slot_sizes;	/* size in bytes of slots */
	objpool_fini_cb         release;	/* resource cleanup callback */
	void                   *context;	/* caller-provided context */
};

#define OBJPOOL_FROM_VMALLOC	(0x800000000)	/* objpool allocated from vmalloc area */
#define OBJPOOL_HAVE_OBJECTS	(0x400000000)	/* objects allocated along with objpool */

/* initialize object pool and pre-allocate objects */
int objpool_init(struct objpool_head *head, int nr_objs, int object_size,
		 gfp_t gfp, void *context, objpool_init_obj_cb objinit,
		 objpool_fini_cb release);

/* allocate an object from objects pool */
void *objpool_pop(struct objpool_head *head);

/* reclaim an object to objects pool */
int objpool_push(void *node, struct objpool_head *head);

/* cleanup the whole object pool (objects including) */
void objpool_fini(struct objpool_head *head);

#endif /* _LINUX_OBJPOOL_H */
